# Rust Playground

A collection of standalone Rust implementations exploring embedded-relevant data structures and algorithms.

## Projects

### ring_buffer

A generic, fixed-size ring buffer (circular buffer) implementation using Rust's const generics.

**Features:**
- Zero heap allocation - size fixed at compile time via const generics
- Compile-time size validation (N > 0)
- Standard queue operations: `push`, `pop`, `is_empty`, `is_full`
- Returns `Result` on push to handle buffer-full condition

**Usage:**
```rust
const BUFFER_SIZE: usize = 7;
let mut buffer = RingBuffer::<usize, BUFFER_SIZE>::new();

buffer.push(42)?;  // Returns Ok(()) or Err(BufferFullError)
let value = buffer.pop();  // Returns Option<T>
```

**Why Ring Buffers Matter in Embedded:**
- Fixed memory footprint (no dynamic allocation)
- Efficient producer-consumer patterns
- Ideal for interrupt-driven data collection
- Common in UART, sensor data buffering, and audio processing
