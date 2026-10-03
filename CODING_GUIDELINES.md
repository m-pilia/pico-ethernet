# Coding Guidelines

Code quality rules for the Pico 2 Ethernet NIC firmware. These apply to all
firmware and host-testable code under `src/`.

## Ground Rules

| Rule | Rationale | How to apply |
|------|-----------|--------------|
| **No dynamic allocation** | Embedded systems have limited heap; stack allocation is predictable and safer | Use stack allocation, static allocation, or custom allocators if absolutely necessary |
| **No C++ exceptions** | Exception handling adds code size overhead and unpredictable execution paths in embedded | Use error codes, `std::optional`, `std::expected`, or `std::variant` for error handling |
| **Disable C++ exceptions in compiler** | Enforce the no-exceptions rule at compile time | Add `-fno-exceptions` to compiler flags |
| **Assert preconditions** | Catch contract violations (out-of-range indices, wrong sizes) at the point of misuse, with zero cost in release builds | See "Preconditions and Assertions" below |

Host-only unit tests are exempt from the no-dynamic-allocation rule: they never
run on the target, so standard containers such as `std::vector` may build their
inputs. On-target self-tests are not exempt.

## Preconditions and Assertions

Validate caller-supplied preconditions with `assert()` from `<cassert>`.

| Rule | Rationale |
|------|-----------|
| **Assert indices and sizes that the caller must satisfy** | Turns silent undefined behavior (out-of-bounds reads, etc.) into a loud failure in debug builds |
| **Keep asserts `constexpr`-friendly** | A failing `assert` in a constant expression is a *compile error*, so misuse in `constexpr` contexts is caught at build time |
| **Do not assert on external/runtime input** | Input that can legitimately be malformed (parsed strings, received packets) must be handled with error returns, not asserts. Asserts are for *programmer* errors, not data errors |
| **Prefer types that make preconditions unrepresentable** | A fixed-extent `std::span<T, N>` enforces length at compile time, so no runtime assert is needed |

Asserts compile out under `NDEBUG` (release/`opt` builds), so they add no
release-build cost. They are active in `fastbuild`/debug builds, including the
host unit tests.

```cpp
// GOOD: precondition on a caller-supplied index.
constexpr uint8_t byte(size_t index) const {
    assert(index < LENGTH);
    return address_[index];
}

// GOOD: the fixed extent is the precondition; no runtime assert needed.
explicit constexpr MacAddress(std::span<const uint8_t, LENGTH> bytes);

// BAD: asserting on data that can legitimately be malformed.
MacAddress parse(std::string_view s) {
    assert(s.size() == 17);  // wrong: malformed input is not a programmer error
    // ...
}
// Instead return std::optional/std::expected and let the caller decide.
```

## Pointer Usage

| Rule | Rationale | Examples |
|------|-----------|----------|
| **Avoid raw pointers** | Prevents ownership ambiguity, memory leaks, dangling pointers | Use `std::unique_ptr`, `std::span`, references instead |
| **Use references for non-owning** | Clear semantics, no null checks needed, safer | `void func(const Data&)` not `void func(const Data*)` |
| **Use `std::span` for buffers** | Safe, bounds-checked, works with arrays and containers | `void process(std::span<const uint8_t> data)` |
| **Use `std::string_view` for strings** | Avoids copies, safe for substrings | `void log(std::string_view msg)` |
| **Avoid pointer arithmetic** | Error-prone, hard to maintain | Use `std::span`, iterators, or `std::array` instead |

### Exceptions to Pointer Rules

Raw pointers are acceptable only when:
1. Interfacing with C libraries that require them (e.g., TinyUSB callbacks)
2. Wrapping in a class that manages lifetime (RAII)
3. Clearly documented with ownership semantics

**Preferred Pattern:**
```cpp
// GOOD: Use span for buffer parameters
void handle_packet(std::span<const uint8_t> data);

// GOOD: Use reference for single objects
void configure(const Config& config);

// GOOD: Use unique_ptr for owned objects
std::unique_ptr<Device> create_device();

// GOOD: Stack allocation for fixed-size data
std::array<uint8_t, 64> buffer;

// BAD: Raw pointer with unclear ownership
void process(Data* data);  // Who owns this? Can it be null?

// BAD: Dynamic allocation
Data* data = new Data();  // Not allowed

// ACCEPTABLE: Raw pointer in C interop (wrapped)
extern "C" {
    void tinyusb_callback(uint8_t* buffer, uint32_t size);
}

class SafeWrapper {
public:
    void on_tinyusb_event(uint8_t* buffer, uint32_t size) {
        // Wrap C callback, convert to span
        handle_packet(std::span(buffer, size));
    }
private:
    void handle_packet(std::span<const uint8_t> data);
};
```

## Modern C++ Guidelines

| Feature | Usage |
|---------|-------|
| **`std::span`** | Preferred for all buffer/array parameters |
| **Ranges** | Use where they improve readability |
| **`constexpr`** | Use for compile-time computations |
| **Concepts** | Use for template constraints |
| **Designated initializers** | Use for struct initialization |
| **`[[nodiscard]]`** | Use for functions where ignoring return is dangerous |
| **Uniform initialization** | Prefer brace `{}` initialization wherever it is correct; see below |

### Uniform Initialization

Prefer brace-initialization (`{}`) wherever it expresses the same intent, because
it rejects narrowing conversions and value-initializes by default.

```cpp
// GOOD
int count{0};
std::array<uint8_t, 6> address{};   // zero-initialized
MacAddress mac{bytes};
Config config{.timeout_ms = 100};   // designated initializer
uint8_t b{narrow_value};            // narrowing is a compile error, not silent

// AVOID when it changes meaning: braces prefer std::initializer_list overloads.
std::vector<int> v(3);              // three elements -- correct
std::vector<int> v{3};              // one element with value 3 -- different!
```

"Whenever correctly possible" is the operative phrase: use `()` (or the most
explicit form) in the cases where `{}` would select an unintended
`std::initializer_list` constructor or otherwise change semantics.

## Integer Types

| Rule | Rationale |
|------|-----------|
| **Use fixed-width integers** | `std::int32_t`, `std::uint8_t`, etc. give predictable size and overflow across compilers and targets |
| **Qualify with `std::`** | Use the C++ `<cstdint>`/`<cstddef>` names (`std::uint8_t`), not the C-style global aliases (`uint8_t`); the standard only guarantees the `std::`-qualified names |
| **No implementation-defined-width integers** | `int`, `unsigned`, `long`, `short`, `uint`, etc. have platform-dependent width and must not be used for our data |

`std::size_t`, `std::ptrdiff_t`, and `std::intptr_t`/`std::uintptr_t` are fine
where their *purpose* is sizes, indices, or pointer arithmetic — they are sized
for that role even though the width is platform-dependent. `char` remains the
type for character/text data.

```cpp
// GOOD
std::uint8_t octet{0};
std::int32_t interval_ms{500};
for (std::size_t i{0}; i < n; ++i) { ... }

// BAD
int interval_ms = 500;   // implementation-defined width
unsigned count = 0;      // ditto
uint8_t octet = 0;       // unqualified C-style name
```

Exceptions: `int main()` is mandated by the language, and a C library's API
(TinyUSB, Pico SDK) may require its own integer types at the call/callback
boundary — mirror the library's signature there, but keep your own variables
fixed-width and `std::`-qualified.

## Increment and Decrement

Use the **prefix** form (`++i`, `--i`) exclusively; the **postfix** form
(`i++`, `i--`) is not allowed.

| Rule | Rationale |
|------|-----------|
| **Always `++i` / `--i`, never `i++` / `i--`** | Postfix conceptually yields a copy of the old value before modifying; prefix carries no such implication and states the intent (increment/decrement) without a discarded temporary. The codebase uses prefix uniformly, including where the result is unused. |

This applies everywhere, including loop updates (`for (...; ++i)`) and array
indexing: write `arr[pos] = x; ++pos;` (or an explicit offset such as
`arr[pos + 1]`) instead of `arr[pos++] = x;`.

```cpp
// GOOD
for (std::size_t i{0}; i < n; ++i) { ... }
buf[pos] = value;
++pos;

// BAD
for (std::size_t i{0}; i < n; i++) { ... }  // postfix loop update
buf[pos++] = value;                          // postfix in index
```

## Compiler Configuration

**Required Flags for Embedded Target:**
```bazel
# In toolchain configuration
cxx_flag = "-fno-exceptions"
cxx_flag = "-fno-rtti"
# No dynamic allocation enforcement is handled by code review
```

## Naming Conventions

| Element | Convention | Examples |
|---------|------------|----------|
| **Namespace** | Single flat namespace | `pico_ethernet` (no nested namespaces, except `detail`; see below) |
| **Classes/Structs** | `UpperCamelCase` | `MacAddress`, `UsbNetDevice` |
| **Methods/Functions** | `snake_case` | `parse()`, `to_string()`, `initialize()` |
| **Variables** | `snake_case` | `mac_address`, `config_`, `is_connected` |
| **Constants** | `UPPER_SNAKE_CASE` | `DEFAULT_MAC`, `MAX_BUFFER_SIZE` |
| **Enumerators** | `UpperCamelCase` | `Duplex::Half`, `FrameError::BadFcs` |
| **Template Parameters** | `UpperCamelCase` | `typename T`, `class Descriptor` |

A nested `detail` namespace is allowed for implementation helpers that must live
at namespace scope (e.g. a table shared by header-only functions) but are not
part of the public API.

## Comment Rules

| Rule | Rationale | Examples |
|------|-----------|----------|
| **No obvious Doxygen** | Comments should add information, not restate the obvious | Bad: `/// @brief Get the MAC address` when method is `get_mac_address()` |
| **Focus on why, not what** | Explain reasoning, not behavior that's clear from code | Good: `// Locally administered address - bit 1 of first byte` |
| **Use for non-trivial info** | Document invariants, assumptions, edge cases | Good: `// Must be called before start()` |
| **Avoid redundant comments** | If the code is clear, no comment needed | Bad: `// Increment i` before `i++` |

**Bad Examples (Remove):**
```cpp
/// @brief Constructor
MacAddress();

/// @brief Get the MAC address as bytes
/// @return The MAC address bytes
const Bytes& bytes() const;

/// @brief Parse a MAC address from a string
/// @param mac_str The string to parse
static std::optional<MacAddress> parse(std::string_view mac_str);
```

**Good Examples (Keep):**
```cpp
// One slot stays free to tell full from empty, so N slots queue N-1 frames.
static constexpr std::size_t TX_QUEUE_SIZE{5};
```
