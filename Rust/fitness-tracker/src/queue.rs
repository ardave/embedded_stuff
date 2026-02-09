use core::ffi::c_void;
use core::marker::PhantomData;
use core::mem;

use esp_idf_svc::sys::{
    vQueueDelete, xQueueGenericCreate, xQueueGenericSend, xQueueReceive, BaseType_t,
    QueueHandle_t,
};

const QUEUE_TYPE_BASE: u8 = 0;
const QUEUE_SEND_TO_BACK: BaseType_t = 0;
const PD_TRUE: BaseType_t = 1;
const PORT_MAX_DELAY: u32 = u32::MAX;

pub struct FreeRtosQueue<T> {
    handle: QueueHandle_t,
    _marker: PhantomData<T>,
}

// FreeRTOS queues are thread-safe by design (internal critical sections).
unsafe impl<T: Send> Send for FreeRtosQueue<T> {}
unsafe impl<T: Send> Sync for FreeRtosQueue<T> {}

impl<T: Copy> FreeRtosQueue<T> {
    pub fn new(length: usize) -> Result<Self, &'static str> {
        let handle = unsafe {
            xQueueGenericCreate(length as u32, mem::size_of::<T>() as u32, QUEUE_TYPE_BASE)
        };
        if handle.is_null() {
            Err("Failed to create FreeRTOS queue")
        } else {
            Ok(Self {
                handle,
                _marker: PhantomData,
            })
        }
    }

    pub fn send(&self, item: &T, timeout_ticks: u32) -> Result<(), &'static str> {
        let ret = unsafe {
            xQueueGenericSend(
                self.handle,
                item as *const T as *const c_void,
                timeout_ticks,
                QUEUE_SEND_TO_BACK,
            )
        };
        if ret == PD_TRUE {
            Ok(())
        } else {
            Err("Queue send failed (full)")
        }
    }

    pub fn try_send(&self, item: &T) -> Result<(), &'static str> {
        self.send(item, 0)
    }

    pub fn recv(&self, timeout_ticks: u32) -> Result<T, &'static str> {
        let mut item = unsafe { mem::zeroed::<T>() };
        let ret = unsafe {
            xQueueReceive(
                self.handle,
                &mut item as *mut T as *mut c_void,
                timeout_ticks,
            )
        };
        if ret == PD_TRUE {
            Ok(item)
        } else {
            Err("Queue receive timed out")
        }
    }

    pub fn recv_blocking(&self) -> T {
        self.recv(PORT_MAX_DELAY)
            .expect("Queue recv with portMAX_DELAY should not fail")
    }
}

impl<T> Drop for FreeRtosQueue<T> {
    fn drop(&mut self) {
        unsafe {
            vQueueDelete(self.handle);
        }
    }
}
