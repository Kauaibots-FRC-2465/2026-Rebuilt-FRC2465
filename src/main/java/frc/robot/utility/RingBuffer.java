package frc.robot.utility;

/**
 * A fixed-size buffer that reuses objects to avoid Garbage Collection.
 * Index [0] always returns the most recently updated element.
 * Call getScratchpad(), fill in the data, then commit it using add().
 */
public class RingBuffer<T> {
    private final T[] buffer;
    private int head = 0; // Points to the newest element
    private int count = 0; // Points to the newest element
    private final int trueCapacity; 

    /**
     * @param capacity How many elements to store.
     * @param supplier A function to create the initial objects (e.g., PinpointBulkRead::new).
     */
    @SuppressWarnings("unchecked")
    public RingBuffer(int capacity, java.util.function.Supplier<T> supplier) {
        trueCapacity = capacity+1;
        buffer = (T[]) new Object[trueCapacity];
        for (int i = 0; i < trueCapacity; i++) {
            buffer[i] = supplier.get();
        }
    }

    /**
     * Returns the next object to be overwritten, one beyond the current head
     * You should update the fields of the returned object.
     */
    public T getScratchpad() {
        return buffer[(head + 1) % trueCapacity]; // Return the current scratchpad
    }

    /**
     * Commits the scratchpad to the buffer.
     */
    public void add() {
        head = (head + 1) % trueCapacity;
        if (count < trueCapacity-1) {   
            count++; // Fill the buffer for the first time, leaving one slot for the scratchpad
        }
    }

    /**
     * Clears the buffer.
     */
    public void clear()
    {
        count = 0;
    }

    /**
     * Access elements relative to the newest.
     * @param index 0 is newest, (size-1) is oldest.
     */
    public T get(int index) {
        if (index < 0 || index >= count) {
            throw new IndexOutOfBoundsException("Index " + index + " out of bounds for size " + count);
        }
        // Formula to look backwards from head with wrap-around
        int target = (head - index + buffer.length) % buffer.length;
        return buffer[target];
    }

    /**
     * Clears the buffer.
     */
    public int size() {
        return count;
    }
}