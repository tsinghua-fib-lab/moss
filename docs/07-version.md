# Version History

## v1.1

- We apply a CUDA memory arena to do dynamic memory allocation in the GPU memory, which make the project been seen as a right CUDA project.
- Based on the memory arena, we implement the checkpoint mechanism to save the simulation state to CPU memory and restore it from CPU memory.
- More efficient database recorder with incompatible APIs.
- Some bug fixes, performance improvements and more Python API.

## From v0.4 to v1.0

That is what we change and why we change it.
- Focus on the microscopic traffic simulation only (vehicle and pedestrian), no crowd in AOI, no bus for more clear code to support community contribution.
- No overlap in junction to avoid deadlock following CBLab's design.
- Can output files with widely-used data format for visualization (visualization is the first for the user to understand the simulation). We choose AVRO as the output format.
- AOI is just as a marker of the starting/ending point of vehicles/pedestrians, no other functions for more clear code.
- Clear code structure and documentation written in English.
