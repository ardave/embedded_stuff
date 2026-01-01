const RING_BUFFER_SIZE: usize = 7;

fn main() {
    println!("Starting...");

    let mut ring_buffer = RingBuffer::<usize, RING_BUFFER_SIZE>::new();

    for x in 0..RING_BUFFER_SIZE + 1 {
        let result = ring_buffer.push(x);
        println!("Pushing {}: {:?}", x, result);
    }

    for x in 0..RING_BUFFER_SIZE + 1 {
        let maybe_element = ring_buffer.pop();
        println!("Popping {}: {:?}", x, maybe_element);
    }

    println!("Done!");
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BufferFullError;

pub struct RingBuffer<T, const N: usize> {
    buffer: [Option<T>; N],
    head: usize,
    tail: usize,
}

impl<T, const N: usize> RingBuffer<T, N> {
    pub fn new() -> Self {
        const { assert!(N > 0, "RingBuffer size must be greater than 0") };
        Self {
            buffer: [const { None }; N],
            head: 0,
            tail: 0,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.tail == self.head && self.buffer[self.tail].is_none()
    }

    pub fn is_full(&self) -> bool {
        self.tail == self.head && self.buffer[self.tail].is_some()
    }

    pub fn push(&mut self, t: T) -> Result<(), BufferFullError> {
        if self.is_full() {
            return Err(BufferFullError);
        }

        self.buffer[self.tail] = Some(t);
        self.tail = (self.tail + 1) % N;
        Ok(())
    }

    pub fn pop(&mut self) -> Option<T> {
        if self.is_empty() {
            return None;
        }

        let maybe_item = self.buffer[self.head].take();

        self.head = (self.head + 1) % N;

        maybe_item
    }
}