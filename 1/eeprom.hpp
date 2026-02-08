#pragma once
#include <cstdint>
#include <cstddef>

/**
 * @file eeprom.hpp
 * @brief EEPROM/NOR abstractions over SPI and a bit-banged SPI bus.
 */

namespace hw {

/**
 * @brief Operation status codes.
 */
enum class Status : uint8_t {
    Ok,         ///< Operation completed successfully.
    InvalidArg, ///< One or more arguments are invalid.
    OutOfRange, ///< Address/length exceeds device capacity.
    IoError,    ///< Low-level SPI/IO error.
    Timeout     ///< Operation timed out.
};

/**
 * @brief Helper to check for Status::Ok.
 * @param s Status code to test.
 * @return true if status equals Status::Ok.
 */
constexpr bool ok(Status s) noexcept { return s == Status::Ok; }

/**
 * @brief Non-owning view of a contiguous memory region.
 * @tparam T Element type.
 */
template<typename T>
class Span {
public:
    /** @brief Construct an empty span. */
    constexpr Span() noexcept : p_(nullptr), n_(0) {}

    /**
     * @brief Construct a span from pointer and length.
     * @param p Pointer to the first element.
     * @param n Number of elements.
     */
    constexpr Span(T* p, size_t n) noexcept : p_(p), n_(n) {}

    /** @brief Pointer to the first element (may be null if size is 0). */
    constexpr T* data() const noexcept { return p_; }

    /** @brief Number of elements in the span. */
    constexpr size_t size() const noexcept { return n_; }

private:
    T* p_;    ///< Pointer to the first element.
    size_t n_;///< Number of elements.
};

/** @brief Mutable byte span. */
using ByteSpan = Span<uint8_t>;
/** @brief Const byte span. */
using ConstByteSpan = Span<const uint8_t>;

/**
 * @brief Abstract digital GPIO pin.
 */
class IGpioPin {
public:
    virtual ~IGpioPin() = default;

    /** @brief Drive the pin output level. */
    virtual void write(bool level) noexcept = 0;

    /** @brief Sample the pin input level. */
    virtual bool read() const noexcept = 0;
};

/**
 * @brief Microsecond delay and time source.
 */
class IDelay {
public:
    virtual ~IDelay() = default;

    /** @brief Busy-wait for a number of microseconds. */
    virtual void delay_us(uint32_t us) noexcept = 0;

    /** @brief Monotonic timestamp in microseconds. */
    virtual uint64_t now_us() const noexcept = 0;
};

/**
 * @brief Full-duplex SPI bus interface.
 * @note Either @p tx or @p rx may be null for transmit-only or receive-only.
 */
class ISpiBus {
public:
    virtual ~ISpiBus() = default;

    /**
     * @brief Transfer bytes on the SPI bus.
     * @param tx Transmit buffer (nullable).
     * @param rx Receive buffer (nullable).
     * @param len Number of bytes to transfer.
     * @return Status code.
     */
    virtual Status transfer(const uint8_t* tx, uint8_t* rx, size_t len) noexcept = 0;
};

/**
 * @brief Bit-banged SPI bus (mode 0, MSB-first).
 * @details Uses GPIO pins and a delay provider to generate SCK.
 */
class BitBangSpiBus final : public ISpiBus {
public:
    /**
     * @brief Construct a bit-banged SPI bus.
     * @param sck Clock pin (idle low).
     * @param mosi MOSI output pin.
     * @param miso MISO input pin.
     * @param delay Delay provider for timing.
     * @param half_period_us Half period of SCK in microseconds.
     */
    BitBangSpiBus(IGpioPin& sck,
                 IGpioPin& mosi,
                 IGpioPin& miso,
                 IDelay& delay,
                 uint32_t half_period_us) noexcept
        : sck_(sck), mosi_(mosi), miso_(miso), delay_(delay), half_period_us_(half_period_us) {
        sck_.write(false);
        mosi_.write(false);
    }

    Status transfer(const uint8_t* tx, uint8_t* rx, size_t len) noexcept override {
        for (size_t i = 0; i < len; ++i) {
            const uint8_t txb = tx ? tx[i] : 0xFFu;
            uint8_t rxb = 0u;

            for (uint8_t mask = 0x80u; mask != 0u; mask >>= 1) {
                const bool out_bit = (txb & mask) != 0u;
                mosi_.write(out_bit);
                clockDelay();

                sck_.write(true);
                clockDelay();
                const bool in_bit = miso_.read();
                rxb = static_cast<uint8_t>((rxb << 1) | (in_bit ? 1u : 0u));

                sck_.write(false);
                clockDelay();
            }

            if (rx) rx[i] = rxb;
        }
        return Status::Ok;
    }

private:
    /** @brief Half-period delay helper. */
    void clockDelay() noexcept {
        if (half_period_us_ != 0u) delay_.delay_us(half_period_us_);
    }

    IGpioPin& sck_;          ///< Clock pin.
    IGpioPin& mosi_;         ///< MOSI pin.
    IGpioPin& miso_;         ///< MISO pin.
    IDelay& delay_;          ///< Delay provider.
    uint32_t half_period_us_;///< Half period in microseconds.
};

/**
 * @brief SPI device wrapper with RAII-controlled CS line.
 */
class SpiDevice {
public:
    /**
     * @brief Construct an SPI device from a bus and CS pin.
     * @param bus SPI bus implementation.
     * @param cs Chip-select pin (active low).
     */
    SpiDevice(ISpiBus& bus, IGpioPin& cs) : bus_(bus), cs_(cs) { cs_.write(true); }

    /**
     * @brief RAII guard that asserts CS low on construction and high on destruction.
     */
    class Guard {
    public:
        /** @brief Assert CS low. */
        explicit Guard(IGpioPin& cs) : cs_(&cs) { cs_->write(false); }

        /** @brief Deassert CS high (if still owned). */
        ~Guard() { release(); }

        Guard(const Guard&) = delete;
        Guard& operator=(const Guard&) = delete;

        /** @brief Move constructor (transfers ownership). */
        Guard(Guard&& other) noexcept : cs_(other.cs_) { other.cs_ = nullptr; }

        /** @brief Move assignment (transfers ownership). */
        Guard& operator=(Guard&& other) noexcept {
            if (this != &other) {
                release();
                cs_ = other.cs_;
                other.cs_ = nullptr;
            }
            return *this;
        }

    private:
        /** @brief Release ownership and deassert CS high. */
        void release() noexcept {
            if (cs_) { cs_->write(true); cs_ = nullptr; }
        }

        IGpioPin* cs_; ///< CS pin (nullptr if released).
    };

    /** @brief Select the device (CS low) for a scope. */
    Guard select() noexcept { return Guard(cs_); }

    /**
     * @brief Transfer bytes on the underlying bus.
     * @param tx Transmit buffer (nullable).
     * @param rx Receive buffer (nullable).
     * @param len Number of bytes.
     * @return Status code.
     */
    Status transfer(const uint8_t* tx, uint8_t* rx, size_t len) noexcept {
        return bus_.transfer(tx, rx, len);
    }

private:
    ISpiBus& bus_; ///< SPI bus implementation.
    IGpioPin& cs_; ///< CS pin.
};

/**
 * @brief Common interface for byte-addressable storage.
 */
class IBlockStorage {
public:
    virtual ~IBlockStorage() = default;

    /** @brief Total capacity in bytes. */
    virtual uint32_t capacity() const noexcept = 0;

    /**
     * @brief Read bytes starting at @p addr.
     * @param addr Byte address.
     * @param out Destination buffer.
     * @return Status code.
     */
    virtual Status read(uint32_t addr, ByteSpan out) noexcept = 0;

    /**
     * @brief Write bytes starting at @p addr.
     * @param addr Byte address.
     * @param data Source buffer.
     * @return Status code.
     */
    virtual Status write(uint32_t addr, ConstByteSpan data) noexcept = 0;
};

namespace detail {
/**
 * @brief Validate that [addr, addr+len) fits within capacity.
 * @param addr Start address.
 * @param len Number of bytes.
 * @param capacity Total capacity in bytes.
 * @return true if range is valid.
 */
constexpr bool range_ok(uint32_t addr, size_t len, uint32_t capacity) noexcept {
    if (len > capacity) return false;
    const uint32_t len32 = static_cast<uint32_t>(len);
    return addr <= (capacity - len32);
}
} // namespace detail

/**
 * @brief EEPROM driver for Microchip 25LC040A (SPI).
 * @details Supports bit/byte/array read and write. Writes are split by page.
 */
class Eeprom25LC040A final : public IBlockStorage {
public:
    static constexpr uint32_t kCapacityBytes = 512; ///< Total capacity (bytes).
    static constexpr uint32_t kCapacityBits  = 4096;///< Total capacity (bits).
    static constexpr uint32_t kPageSize      = 16;  ///< EEPROM page size (bytes).

    /**
     * @brief Construct the EEPROM driver.
     * @param dev SPI device wrapper.
     * @param delay Delay provider.
     */
    explicit Eeprom25LC040A(SpiDevice& dev, IDelay& delay) : dev_(dev), delay_(delay) {}

    uint32_t capacity() const noexcept override { return kCapacityBytes; }

    Status read(uint32_t addr, ByteSpan out) noexcept override {
        if ((!out.data() && out.size() != 0)) return Status::InvalidArg;
        if (!detail::range_ok(addr, out.size(), kCapacityBytes)) return Status::OutOfRange;

        uint8_t hdr[3] = {
            CMD_READ,
            static_cast<uint8_t>((addr >> 8) & 0xFFu),
            static_cast<uint8_t>(addr & 0xFFu)
        };

        auto g = dev_.select();
        Status s = dev_.transfer(hdr, nullptr, sizeof(hdr));
        if (!ok(s)) return s;

        return dev_.transfer(nullptr, out.data(), out.size());
    }

    Status write(uint32_t addr, ConstByteSpan data) noexcept override {
        if ((!data.data() && data.size() != 0)) return Status::InvalidArg;
        if (!detail::range_ok(addr, data.size(), kCapacityBytes)) return Status::OutOfRange;

        uint32_t off = 0;
        while (off < data.size()) {
            const uint32_t page_off = (addr + off) % kPageSize;
            const uint32_t room = kPageSize - page_off;
            const uint32_t left = static_cast<uint32_t>(data.size() - off);
            const uint32_t chunk = (left < room) ? left : room;

            Status s = writePage(addr + off, ConstByteSpan(data.data() + off, chunk));
            if (!ok(s)) return s;

            off += chunk;
        }
        return Status::Ok;
    }

    /**
     * @brief Read a single byte at @p addr.
     * @param addr Byte address.
     * @param out Output byte.
     * @return Status code.
     */
    Status readByte(uint32_t addr, uint8_t& out) noexcept {
        return read(addr, ByteSpan(&out, 1));
    }

    /**
     * @brief Write a single byte at @p addr.
     * @param addr Byte address.
     * @param value Byte to write.
     * @return Status code.
     */
    Status writeByte(uint32_t addr, uint8_t value) noexcept {
        return write(addr, ConstByteSpan(&value, 1));
    }

    /**
     * @brief Read a single bit by bit address.
     * @param bit_addr Bit address [0..kCapacityBits-1].
     * @param out Output bit value.
     * @return Status code.
     */
    Status readBit(uint32_t bit_addr, bool& out) noexcept {
        if (bit_addr >= kCapacityBits) return Status::OutOfRange;
        const uint32_t byte_addr = bit_addr / 8;
        const uint32_t bit = bit_addr % 8;
        uint8_t b = 0;
        Status s = read(byte_addr, ByteSpan(&b, 1));
        if (!ok(s)) return s;
        out = ((b >> bit) & 1u) != 0;
        return Status::Ok;
    }

    /**
     * @brief Write a single bit by bit address.
     * @param bit_addr Bit address [0..kCapacityBits-1].
     * @param value Bit value to write.
     * @return Status code.
     */
    Status writeBit(uint32_t bit_addr, bool value) noexcept {
        if (bit_addr >= kCapacityBits) return Status::OutOfRange;
        const uint32_t byte_addr = bit_addr / 8;
        const uint32_t bit = bit_addr % 8;

        uint8_t b = 0;
        Status s = read(byte_addr, ByteSpan(&b, 1));
        if (!ok(s)) return s;

        const uint8_t mask = static_cast<uint8_t>(1u << bit);
        b = value ? static_cast<uint8_t>(b | mask) : static_cast<uint8_t>(b & ~mask);

        return write(byte_addr, ConstByteSpan(&b, 1));
    }

private:
    static constexpr uint8_t CMD_READ  = 0x03; ///< Read data command.
    static constexpr uint8_t CMD_WRITE = 0x02; ///< Write data command.
    static constexpr uint8_t CMD_WREN  = 0x06; ///< Write enable command.
    static constexpr uint8_t CMD_RDSR  = 0x05; ///< Read status register command.

    static constexpr uint8_t SR_WIP = 1u << 0; ///< Write-in-progress bit.

    static constexpr uint32_t kPollDelayUs = 50;        ///< Poll delay during write.
    static constexpr uint32_t kTimeoutUs   = 2'000'000; ///< Write timeout.

    /** @brief Send WREN (write enable). */
    Status writeEnable() noexcept {
        auto g = dev_.select();
        const uint8_t cmd = CMD_WREN;
        return dev_.transfer(&cmd, nullptr, 1);
    }

    /** @brief Read the status register. */
    Status readStatus(uint8_t& sr) noexcept {
        auto g = dev_.select();
        const uint8_t tx[2] = { CMD_RDSR, 0xFF };
        uint8_t rx[2] = {};
        Status s = dev_.transfer(tx, rx, 2);
        if (!ok(s)) return s;
        sr = rx[1];
        return Status::Ok;
    }

    /** @brief Poll until device is ready or timeout elapses. */
    Status waitReady(uint32_t timeout_us) noexcept {
        const uint64_t deadline = delay_.now_us() + timeout_us;
        while (delay_.now_us() < deadline) {
            uint8_t sr = 0;
            Status s = readStatus(sr);
            if (!ok(s)) return s;
            if ((sr & SR_WIP) == 0) return Status::Ok;
            delay_.delay_us(kPollDelayUs);
        }
        return Status::Timeout;
    }

    /**
     * @brief Write a single EEPROM page (must not cross page boundary).
     * @param addr Page-aligned start address.
     * @param data Data to write (<= kPageSize).
     */
    Status writePage(uint32_t addr, ConstByteSpan data) noexcept {
        if (((addr % kPageSize) + data.size()) > kPageSize) return Status::InvalidArg;

        Status s = writeEnable();
        if (!ok(s)) return s;

        uint8_t hdr[3] = {
            CMD_WRITE,
            static_cast<uint8_t>((addr >> 8) & 0xFFu),
            static_cast<uint8_t>(addr & 0xFFu)
        };

        {
            auto g = dev_.select();
            s = dev_.transfer(hdr, nullptr, sizeof(hdr));
            if (!ok(s)) return s;

            s = dev_.transfer(data.data(), nullptr, data.size());
            if (!ok(s)) return s;
        }

        return waitReady(kTimeoutUs);
    }

    SpiDevice& dev_; ///< SPI device wrapper.
    IDelay& delay_;  ///< Delay provider.
};

/**
 * @brief NOR flash driver for Winbond W25Q128 (SPI).
 *
 * @note Differences compared to EEPROM (25LC040A):
 * - Bits can only change from 1 to 0 during program; erase is required to set bits back to 1.
 * - Erase granularity is a sector/block (e.g., 4 KiB); adds erase commands and longer busy times.
 * - Larger capacity uses 24-bit addresses (3 bytes) and larger page size (256 bytes).
 * - Additional status/configuration registers may exist.
 */
class NorW25Q128 final : public IBlockStorage {
public:
    static constexpr uint32_t kCapacityBytes = 16u * 1024u * 1024u; ///< 128 Mbit = 16 MiB.
    static constexpr uint32_t kPageSize      = 256;                 ///< Page program size.
    static constexpr uint32_t kSectorSize    = 4096;                ///< Sector erase size.

    /**
     * @brief Construct the NOR flash driver.
     * @param dev SPI device wrapper.
     * @param delay Delay provider.
     */
    explicit NorW25Q128(SpiDevice& dev, IDelay& delay) : dev_(dev), delay_(delay) {}

    uint32_t capacity() const noexcept override { return kCapacityBytes; }

    Status read(uint32_t addr, ByteSpan out) noexcept override {
        if ((!out.data() && out.size() != 0)) return Status::InvalidArg;
        if (!detail::range_ok(addr, out.size(), kCapacityBytes)) return Status::OutOfRange;

        uint8_t hdr[4] = {
            CMD_READ,
            static_cast<uint8_t>((addr >> 16) & 0xFFu),
            static_cast<uint8_t>((addr >> 8) & 0xFFu),
            static_cast<uint8_t>(addr & 0xFFu)
        };

        auto g = dev_.select();
        Status s = dev_.transfer(hdr, nullptr, sizeof(hdr));
        if (!ok(s)) return s;

        return dev_.transfer(nullptr, out.data(), out.size());
    }

    /**
     * @brief Program bytes starting at @p addr.
     * @details Caller must ensure the region is erased (all bits 1).
     */
    Status write(uint32_t addr, ConstByteSpan data) noexcept override {
        if ((!data.data() && data.size() != 0)) return Status::InvalidArg;
        if (!detail::range_ok(addr, data.size(), kCapacityBytes)) return Status::OutOfRange;

        uint32_t off = 0;
        while (off < data.size()) {
            const uint32_t page_off = (addr + off) % kPageSize;
            const uint32_t room = kPageSize - page_off;
            const uint32_t left = static_cast<uint32_t>(data.size() - off);
            const uint32_t chunk = (left < room) ? left : room;

            Status s = programPage(addr + off, ConstByteSpan(data.data() + off, chunk));
            if (!ok(s)) return s;

            off += chunk;
        }
        return Status::Ok;
    }

    /**
     * @brief Erase a 4 KiB sector.
     * @param addr Sector-aligned address.
     * @return Status code.
     */
    Status eraseSector(uint32_t addr) noexcept {
        if (addr % kSectorSize != 0) return Status::InvalidArg;
        if (!detail::range_ok(addr, kSectorSize, kCapacityBytes)) return Status::OutOfRange;

        Status s = writeEnable();
        if (!ok(s)) return s;

        uint8_t hdr[4] = {
            CMD_SE,
            static_cast<uint8_t>((addr >> 16) & 0xFFu),
            static_cast<uint8_t>((addr >> 8) & 0xFFu),
            static_cast<uint8_t>(addr & 0xFFu)
        };

        {
            auto g = dev_.select();
            s = dev_.transfer(hdr, nullptr, sizeof(hdr));
            if (!ok(s)) return s;
        }

        return waitReady(kEraseTimeoutUs);
    }

    /**
     * @brief Erase the entire chip.
     * @return Status code.
     */
    Status eraseChip() noexcept {
        Status s = writeEnable();
        if (!ok(s)) return s;

        const uint8_t cmd = CMD_CE;
        {
            auto g = dev_.select();
            s = dev_.transfer(&cmd, nullptr, 1);
            if (!ok(s)) return s;
        }

        return waitReady(kChipEraseTimeoutUs);
    }

private:
    static constexpr uint8_t CMD_READ = 0x03; ///< Read data command.
    static constexpr uint8_t CMD_PP   = 0x02; ///< Page program command.
    static constexpr uint8_t CMD_WREN = 0x06; ///< Write enable command.
    static constexpr uint8_t CMD_RDSR = 0x05; ///< Read status register command.
    static constexpr uint8_t CMD_SE   = 0x20; ///< 4 KiB sector erase command.
    static constexpr uint8_t CMD_CE   = 0xC7; ///< Chip erase command.

    static constexpr uint8_t SR_WIP = 1u << 0; ///< Write-in-progress bit.

    static constexpr uint32_t kPollDelayUs       = 50;        ///< Poll delay during busy.
    static constexpr uint32_t kProgramTimeoutUs  = 2'000'000; ///< Page program timeout.
    static constexpr uint32_t kEraseTimeoutUs    = 6'000'000; ///< Sector erase timeout.
    static constexpr uint32_t kChipEraseTimeoutUs= 60'000'000;///< Chip erase timeout.

    /** @brief Send WREN (write enable). */
    Status writeEnable() noexcept {
        auto g = dev_.select();
        const uint8_t cmd = CMD_WREN;
        return dev_.transfer(&cmd, nullptr, 1);
    }

    /** @brief Read the status register. */
    Status readStatus(uint8_t& sr) noexcept {
        auto g = dev_.select();
        const uint8_t tx[2] = { CMD_RDSR, 0xFF };
        uint8_t rx[2] = {};
        Status s = dev_.transfer(tx, rx, 2);
        if (!ok(s)) return s;
        sr = rx[1];
        return Status::Ok;
    }

    /** @brief Poll until device is ready or timeout elapses. */
    Status waitReady(uint32_t timeout_us) noexcept {
        const uint64_t deadline = delay_.now_us() + timeout_us;
        while (delay_.now_us() < deadline) {
            uint8_t sr = 0;
            Status s = readStatus(sr);
            if (!ok(s)) return s;
            if ((sr & SR_WIP) == 0) return Status::Ok;
            delay_.delay_us(kPollDelayUs);
        }
        return Status::Timeout;
    }

    /**
     * @brief Program a single NOR flash page (must not cross page boundary).
     * @param addr Start address.
     * @param data Data to program (<= kPageSize).
     */
    Status programPage(uint32_t addr, ConstByteSpan data) noexcept {
        if (((addr % kPageSize) + data.size()) > kPageSize) return Status::InvalidArg;

        Status s = writeEnable();
        if (!ok(s)) return s;

        uint8_t hdr[4] = {
            CMD_PP,
            static_cast<uint8_t>((addr >> 16) & 0xFFu),
            static_cast<uint8_t>((addr >> 8) & 0xFFu),
            static_cast<uint8_t>(addr & 0xFFu)
        };

        {
            auto g = dev_.select();
            s = dev_.transfer(hdr, nullptr, sizeof(hdr));
            if (!ok(s)) return s;

            s = dev_.transfer(data.data(), nullptr, data.size());
            if (!ok(s)) return s;
        }

        return waitReady(kProgramTimeoutUs);
    }

    SpiDevice& dev_; ///< SPI device wrapper.
    IDelay& delay_;  ///< Delay provider.
};

} // namespace hw
