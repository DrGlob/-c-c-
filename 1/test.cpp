#include "eeprom.hpp"
#include <vector>
#include <cstdio>
#include <cstdlib>

using namespace hw;

struct MockPin : IGpioPin {
    bool level = true;
    std::vector<bool> history;
    void write(bool l) noexcept override { level = l; history.push_back(l); }
    bool read() const noexcept override { return level; }
};

struct MockDelay : IDelay {
    uint64_t t = 0;
    void delay_us(uint32_t us) noexcept override { t += us; }
    uint64_t now_us() const noexcept override { return t; }
};

struct MockSpiBus : ISpiBus {
    struct Call { std::vector<uint8_t> tx; size_t len = 0; };
    std::vector<Call> calls;

    int rdsr_count = 0;

    Status transfer(const uint8_t* tx, uint8_t* rx, size_t len) noexcept override {
        Call c; c.len = len; c.tx.reserve(len);
        for (size_t i = 0; i < len; i++) c.tx.push_back(tx ? tx[i] : 0xFF);
        calls.push_back(c);

        // Script RDSR: first poll WIP=1, then WIP=0, then always 0
        if (rx && len == 2 && tx && tx[0] == 0x05) {
            rx[0] = 0x00;
            rx[1] = (rdsr_count == 0) ? 0x01 : 0x00;
            rdsr_count++;
        }
        return Status::Ok;
    }
};

static void expect(bool cond, const char* msg) {
    if (!cond) { std::printf("FAIL: %s\n", msg); std::exit(1); }
}

static size_t find_next_wren(const MockSpiBus& spi, size_t start) {
    for (size_t i = start; i < spi.calls.size(); i++) {
        if (spi.calls[i].len == 1 && spi.calls[i].tx.size() == 1 && spi.calls[i].tx[0] == 0x06) return i;
    }
    return (size_t)-1;
}

int main() {
    MockSpiBus spi;
    MockPin cs;
    MockDelay delay;

    SpiDevice dev(spi, cs);
    Eeprom25LC040A e(dev, delay);

    // crosses page boundary: addr=0x000F, len=3
    const uint8_t data[3] = { 0xAA, 0xBB, 0xCC };
    Status s = e.write(0x000F, ConstByteSpan(data, 3));
    expect(ok(s), "write should succeed");
    expect(spi.calls.size() >= 9, "expected multiple SPI transfers (2 chunks + polls)");

    // first WREN
    expect(spi.calls[0].tx[0] == 0x06, "first must be WREN");

    // first WRITE header addr=0x000F
    expect(spi.calls[1].len == 3 && spi.calls[1].tx[0] == 0x02, "WRITE header #1");
    expect(spi.calls[1].tx[1] == 0x00 && spi.calls[1].tx[2] == 0x0F, "WRITE addr #1 0x000F");

    // first data chunk is 1 byte AA
    expect(spi.calls[2].len == 1 && spi.calls[2].tx[0] == 0xAA, "data chunk #1 must be AA");

    // should poll RDSR at least once
    bool saw_rdsr = false;
    for (size_t i = 3; i < spi.calls.size(); i++) {
        if (spi.calls[i].len == 2 && spi.calls[i].tx[0] == 0x05) { saw_rdsr = true; break; }
    }
    expect(saw_rdsr, "must poll RDSR");

    // find second WREN
    size_t wren2 = find_next_wren(spi, 3);
    expect(wren2 != (size_t)-1, "second WREN must exist");

    // second WRITE header addr=0x0010
    expect(spi.calls[wren2 + 1].len == 3 && spi.calls[wren2 + 1].tx[0] == 0x02, "WRITE header #2");
    expect(spi.calls[wren2 + 1].tx[1] == 0x00 && spi.calls[wren2 + 1].tx[2] == 0x10, "WRITE addr #2 0x0010");

    // second data chunk is 2 bytes BB CC
    expect(spi.calls[wren2 + 2].len == 2, "data chunk #2 length");
    expect(spi.calls[wren2 + 2].tx[0] == 0xBB && spi.calls[wren2 + 2].tx[1] == 0xCC, "data chunk #2 bytes");

    // CS toggling: initial high + many low/high transitions
    expect(cs.history.size() > 1, "CS should toggle");
    expect(cs.history[0] == true, "CS initial high");

    std::printf("PASS\n");
    return 0;
}
