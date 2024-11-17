# MemManger

Wrap CUDA unified memory management functions to provide a more user-friendly interface.

Due to the difficulty to do dynamic device memory management in CUDA, we provide MemManager as a memory pool to manage device memory.
The manager will allocate a large chunk of memory at the beginning and then allocate memory from this chunk.

#### TODO

- [ ] Try better memory management strategies to support more dynamic actions.
- [ ] Allow free memory in the middle of the chunk.
