#include <systemc>

#include <array>
#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using sc_core::SC_NS;
using sc_core::SC_US;
using sc_core::sc_time;

namespace {

constexpr std::uint8_t CMD_READ_ROM = 0x33;
constexpr std::uint8_t CMD_SKIP_ROM = 0xcc;
constexpr std::uint8_t CMD_CONVERT_T = 0x44;
constexpr std::uint8_t CMD_READ_SCRATCHPAD = 0xbe;

struct TimingSpec {
    sc_time reset_low_min{480, SC_US};
    sc_time reset_high_min{480, SC_US};
    sc_time presence_high_min{15, SC_US};
    sc_time presence_high_max{60, SC_US};
    sc_time presence_low_min{60, SC_US};
    sc_time presence_low_max{240, SC_US};
    sc_time slot_min{60, SC_US};
    sc_time slot_max{120, SC_US};
    sc_time recovery_min{1, SC_US};
    sc_time write0_low_min{60, SC_US};
    sc_time write0_low_max{120, SC_US};
    sc_time write1_low_min{1, SC_US};
    sc_time write1_low_max{15, SC_US};
    sc_time read_data_valid_max{15, SC_US};
    sc_time convert_12bit_max{750000, SC_US};
};

struct PhysicalParams {
    double vpu_v = 3.3;
    double vil_max_v = 0.8;
    double vih_min_v = 2.2;
    double r_pullup_ohm = 4700.0;
    double r_strong_pullup_ohm = 100.0;
    double c_bus_pf = 125.0;
};

struct TraceEvent {
    std::string layer;
    sc_time t;
    std::string event;
    std::string detail;
};

std::string hex_u8(std::uint8_t value) {
    std::ostringstream os;
    os << "0x" << std::hex << std::nouppercase << std::setw(2)
       << std::setfill('0') << static_cast<unsigned>(value);
    return os.str();
}

std::uint8_t crc8_byte(std::uint8_t crc_in, std::uint8_t data_in) {
    std::uint8_t crc = crc_in;
    std::uint8_t data = data_in;
    for (int bit_idx = 0; bit_idx < 8; ++bit_idx) {
        if (((crc ^ data) & 0x01U) != 0U) {
            crc = static_cast<std::uint8_t>((crc >> 1U) ^ 0x8cU);
        } else {
            crc = static_cast<std::uint8_t>(crc >> 1U);
        }
        data = static_cast<std::uint8_t>(data >> 1U);
    }
    return crc;
}

class Ds18b20PhysicalLine {
  public:
    explicit Ds18b20PhysicalLine(PhysicalParams params) : params_(params) {}

    sc_time rise_time_to_vih(bool strong_pullup) const {
        const double resistance = strong_pullup ? params_.r_strong_pullup_ohm : params_.r_pullup_ohm;
        const double capacitance_f = params_.c_bus_pf * 1.0e-12;
        if (params_.vih_min_v >= params_.vpu_v) {
            throw std::runtime_error("VIH threshold must be lower than VPU");
        }
        const double seconds = -resistance * capacitance_f *
                               std::log(1.0 - (params_.vih_min_v / params_.vpu_v));
        return sc_time(seconds, sc_core::SC_SEC);
    }

    double voltage_after_release(sc_time elapsed, bool strong_pullup) const {
        const double resistance = strong_pullup ? params_.r_strong_pullup_ohm : params_.r_pullup_ohm;
        const double capacitance_f = params_.c_bus_pf * 1.0e-12;
        const double seconds = elapsed.to_seconds();
        return params_.vpu_v * (1.0 - std::exp(-seconds / (resistance * capacitance_f)));
    }

    const PhysicalParams &params() const { return params_; }

  private:
    PhysicalParams params_;
};

class Ds18b20TransactionModel {
  public:
    Ds18b20TransactionModel(std::uint64_t rom_code, int temp_c_x16)
        : rom_code_(rom_code), temp_c_x16_(temp_c_x16) {
        refresh_scratchpad();
    }

    std::uint64_t rom_code() const { return rom_code_; }

    std::vector<std::uint8_t> accept_command(std::uint8_t command) {
        switch (command) {
        case CMD_SKIP_ROM:
            rom_skipped_ = true;
            return {};
        case CMD_CONVERT_T:
            conversion_done_ = true;
            refresh_scratchpad();
            return {};
        case CMD_READ_SCRATCHPAD:
            return std::vector<std::uint8_t>(scratchpad_.begin(), scratchpad_.end());
        case CMD_READ_ROM:
            return rom_bytes();
        default:
            return {};
        }
    }

    std::vector<std::uint8_t> rom_bytes() const {
        std::vector<std::uint8_t> bytes;
        for (int idx = 0; idx < 8; ++idx) {
            bytes.push_back(static_cast<std::uint8_t>((rom_code_ >> (8 * idx)) & 0xffU));
        }
        return bytes;
    }

    const std::array<std::uint8_t, 9> &scratchpad() const { return scratchpad_; }

  private:
    void refresh_scratchpad() {
        const std::int16_t raw_temp = static_cast<std::int16_t>(temp_c_x16_);
        scratchpad_[0] = static_cast<std::uint8_t>(raw_temp & 0xff);
        scratchpad_[1] = static_cast<std::uint8_t>((raw_temp >> 8) & 0xff);
        scratchpad_[2] = 0x4b;
        scratchpad_[3] = 0x46;
        scratchpad_[4] = 0x7f;
        scratchpad_[5] = 0xff;
        scratchpad_[6] = 0x0c;
        scratchpad_[7] = 0x10;
        std::uint8_t crc = 0;
        for (int idx = 0; idx < 8; ++idx) {
            crc = crc8_byte(crc, scratchpad_[idx]);
        }
        scratchpad_[8] = crc;
    }

    std::uint64_t rom_code_;
    int temp_c_x16_;
    bool conversion_done_ = true;
    bool rom_skipped_ = false;
    std::array<std::uint8_t, 9> scratchpad_{};
};

class OneWireLinkModel {
  public:
    OneWireLinkModel(TimingSpec timing, Ds18b20PhysicalLine physical, Ds18b20TransactionModel sensor)
        : timing_(timing), physical_(physical), sensor_(sensor) {}

    void reset_presence() {
        trace("link", "reset_low", "master drives low for 500 us");
        now_ += sc_time(500, SC_US);
        trace("physical", "release", "weak pull-up tVIH=" + std::to_string(physical_.rise_time_to_vih(false).to_seconds() * 1.0e6) + "us");
        now_ += physical_.rise_time_to_vih(false);
        now_ += sc_time(30, SC_US);
        trace("link", "presence_low", "sensor presence pulse 120 us");
        now_ += sc_time(120, SC_US);
        trace("link", "presence_release", "sensor releases bus");
        now_ += physical_.rise_time_to_vih(false);
    }

    void write_byte(std::uint8_t value) {
        trace("link", "write_byte_begin", hex_u8(value));
        for (int bit_idx = 0; bit_idx < 8; ++bit_idx) {
            const bool bit = ((value >> bit_idx) & 1U) != 0U;
            const sc_time low_time = bit ? sc_time(5, SC_US) : sc_time(65, SC_US);
            trace("link", bit ? "write1_slot" : "write0_slot", "bit=" + std::to_string(bit_idx));
            now_ += low_time;
            now_ += physical_.rise_time_to_vih(false);
            now_ += sc_time(90, SC_US) - low_time;
            now_ += timing_.recovery_min;
        }
        pending_read_ = sensor_.accept_command(value);
        pending_read_idx_ = 0;
        trace("transaction", "command_accept", hex_u8(value) + " response_bytes=" + std::to_string(pending_read_.size()));
    }

    std::uint8_t read_byte() {
        std::uint8_t value = 0;
        const std::uint8_t source = (pending_read_idx_ < pending_read_.size()) ? pending_read_[pending_read_idx_++] : 0xffU;
        trace("link", "read_byte_begin", "source=" + hex_u8(source));
        for (int bit_idx = 0; bit_idx < 8; ++bit_idx) {
            const bool bit = ((source >> bit_idx) & 1U) != 0U;
            trace("link", bit ? "read1_slot" : "read0_slot", "bit=" + std::to_string(bit_idx));
            now_ += sc_time(5, SC_US);
            if (bit) {
                now_ += physical_.rise_time_to_vih(false);
            } else {
                now_ += sc_time(10, SC_US);
            }
            const double sample_v = bit ? physical_.voltage_after_release(sc_time(10, SC_US), false) : 0.0;
            if (bit && sample_v < physical_.params().vih_min_v) {
                trace("physical", "read1_margin_fail", "sample_v=" + std::to_string(sample_v));
            }
            if (bit) {
                value |= static_cast<std::uint8_t>(1U << bit_idx);
            }
            now_ += sc_time(90, SC_US);
            now_ += timing_.recovery_min;
        }
        trace("link", "read_byte_end", hex_u8(value));
        return value;
    }

    void run_temperature_sequence() {
        reset_presence();
        write_byte(CMD_SKIP_ROM);
        write_byte(CMD_CONVERT_T);
        now_ += sc_time(1000, SC_US);
        reset_presence();
        write_byte(CMD_SKIP_ROM);
        write_byte(CMD_READ_SCRATCHPAD);
        for (int idx = 0; idx < 9; ++idx) {
            (void)read_byte();
        }
    }

    void run_rom_sequence() {
        reset_presence();
        write_byte(CMD_READ_ROM);
        for (int idx = 0; idx < 8; ++idx) {
            (void)read_byte();
        }
    }

    void write_trace_csv(const std::string &path) const {
        std::ofstream out(path);
        if (!out) {
            throw std::runtime_error("cannot open output trace " + path);
        }
        out << "time_us,layer,event,detail\n";
        for (const auto &event : trace_) {
            out << std::fixed << std::setprecision(6) << event.t.to_seconds() * 1.0e6
                << "," << event.layer << "," << event.event << ",\"" << event.detail << "\"\n";
        }
    }

    void append_rc_sweep(const std::string &path) const {
        std::ofstream out(path, std::ios::app);
        const std::array<double, 6> c_values_pf{{25.0, 50.0, 125.0, 250.0, 500.0, 1000.0}};
        for (double c_pf : c_values_pf) {
            PhysicalParams params = physical_.params();
            params.c_bus_pf = c_pf;
            Ds18b20PhysicalLine line(params);
            out << std::fixed << std::setprecision(6)
                << "0.000000,physical,rc_sweep,\"c_pf=" << c_pf
                << " weak_tVIH_us=" << line.rise_time_to_vih(false).to_seconds() * 1.0e6
                << " strong_tVIH_us=" << line.rise_time_to_vih(true).to_seconds() * 1.0e6
                << "\"\n";
        }
    }

  private:
    void trace(const std::string &layer, const std::string &event, const std::string &detail) {
        trace_.push_back({layer, now_, event, detail});
    }

    TimingSpec timing_;
    Ds18b20PhysicalLine physical_;
    Ds18b20TransactionModel sensor_;
    sc_time now_{0, SC_US};
    std::vector<std::uint8_t> pending_read_;
    std::size_t pending_read_idx_ = 0;
    std::vector<TraceEvent> trace_;
};

} // namespace

int sc_main(int argc, char **argv) {
    std::string out_path = "ds18b20_systemc_trace.csv";
    if (const char *env_out = std::getenv("DS18B20_TRACE_OUT")) {
        out_path = env_out;
    }
    for (int arg_idx = 1; arg_idx < argc; ++arg_idx) {
        const std::string arg = argv[arg_idx];
        if (arg == "--out" && arg_idx + 1 < argc) {
            out_path = argv[++arg_idx];
        }
    }

    TimingSpec timing;
    PhysicalParams physical_params;
    Ds18b20PhysicalLine physical(physical_params);
    Ds18b20TransactionModel sensor(0x2800000000000100ULL, 25 * 16);
    OneWireLinkModel link(timing, physical, sensor);

    link.run_temperature_sequence();
    link.run_rom_sequence();
    link.write_trace_csv(out_path);
    link.append_rc_sweep(out_path);

    std::cout << "DS18B20 SystemC model wrote " << out_path << "\n";
    std::cout << "weak pull-up tVIH(us)=" << physical.rise_time_to_vih(false).to_seconds() * 1.0e6
              << " strong pull-up tVIH(us)=" << physical.rise_time_to_vih(true).to_seconds() * 1.0e6
              << "\n";
    return 0;
}
