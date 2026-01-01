#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct {
    int *buffer;
    size_t head;
    size_t tail;
    size_t capacity;
    size_t count;
} RingBuffer;

bool rb_init(RingBuffer *rb, size_t capacity) {
    if (rb == NULL) return false;
    if (capacity == 0) return false;
    if (capacity > SIZE_MAX / sizeof(int)) return false;

    rb->buffer = malloc(capacity * sizeof(int));
    if (rb->buffer == NULL) {
        return false;
    }
    rb->head = 0;
    rb->tail = 0;
    rb->capacity = capacity;
    rb->count = 0;
    return true;
}

void rb_destroy(RingBuffer *rb) {
    if (rb == NULL) {
        return false;
    }
     free(rb->buffer);
     rb->buffer = NULL;
}

bool rb_is_empty(const RingBuffer *rb) {
    return rb->count == 0;
}

bool rb_is_full(const RingBuffer *rb) {
    return rb->count == rb->capacity;
}

bool rb_push(RingBuffer *rb, int value) {
    if (rb_is_full(rb)) return false;

    rb->buffer[rb->head] = value;
    rb->head = (rb->head + 1) % rb->capacity;
    rb->count++;
    return true;
}

bool rb_pop(RingBuffer *rb, int *out) {
    if (rb_is_empty(rb)) return false;

    *out = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % rb->capacity;
    rb->count--;
    return true;
}

int main(void) {
    printf("Starting...\n");

    int size = 7;

    RingBuffer rb;
    if (rb_init(&rb, size) == false) {
        return 1;
    }

    for (int value = 0; value < size + 1; value++) {
        bool success = rb_push(&rb, value);
        printf("Pushing value %i, succeeded? %d\n", value, success);
    }

    for (int idx = 0; idx < size + 1; idx++) {
        int value;
        bool success = rb_pop(&rb, &value);
        printf("Popping index %i, succeeded? %d\n", idx, success);
    }

    rb_destroy(&rb);
    printf("Done!\n");
    return 0;
}


