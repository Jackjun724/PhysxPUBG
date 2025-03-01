# Physx PUBG Raycast Trace

## WARNING
To all those who steal and sell my source code: Where are your parents?

## Project
[Link](https://github.com/Jackjun724/PhysxPUBG)

## Project Overview

This project is a tool that dynamically reconstructs the game physics model externally by reading the Physx memory segments from the PUBG game process. It utilizes external memory reading techniques to obtain real-time physical model data from the game and visualizes it with ray tracing capabilities in a separate window.

### Core Features

- **Memory Reading**: Access PUBG game process memory through DMA to read Physx physics engine related data
- **Physics Model Reconstruction**: Real-time reconstruction of triangle meshes, height fields, and rigid body physics models from the game world
- **Ray Tracing**: Provides ray tracing functionality to detect collisions between line of sight and game world objects
- **Visualization**: Uses DirectX and ImGui to visualize the reconstructed models
- **Model Export**: Supports exporting the acquired 3D models to OBJ format files. You need to aim your crosshair at the Shape, and when it's rendered on screen, press F1. The model's obj file will be saved in the running directory.

### Technical Characteristics

- Utilizes the Embree engine for efficient ray tracing calculations
- Multi-threaded parallel processing of different types of physical objects (static objects, dynamic rigid bodies, height fields)
- Accesses protected game data through memory decryption
- Supports dynamic loading and unloading of model data based on visible range to optimize performance

### Main Components

- **VisibleScene**: Manages the scene containing various physical objects
- **LineTrace**: Provides ray tracing functionality for detecting interactions between line of sight and objects
- **Physx**: Interface for interacting with the game's Physx engine data structures
- **VisibleCheck**: Handles scene updates and visibility detection processing

## Roadmap
- [x] Rendering of static rigid bodies
- [x] Rendering of dynamic rigid bodies
- [x] Rendering of dynamically loaded height fields
- [x] Destructible static rigid bodies in PUBG, such as buildings in Sanhok map
- [x] Game-specific shaped patches, such as wire mesh fences
- [ ] Fix occasional memory errors
- [ ] Fix mesh rendering issues that frequently occur on low-memory host computers


## Build & Run
The project requires Embree-related DLLs and DMA-related DLLs to run. For security considerations, please import these dependencies yourself.
- embree4.dll
- tbb12.dll
- leechcore.dll
- vmm.dll
- FTD3XX.dll


## About mesh patch
First, you need to have the saved model file, which is named as VertiesNum_IndicesNum.obj. I usually recommend editing it in MeshLab, where you should delete the unwanted Faces but keep all vertices intact. This will maintain the index data of the reconstructed model. Then export it as a JSON file which will have the new vertex indices. Copy it to MeshPatcher.h file!

## Patch Video
[Video](https://youtu.be/tX1Ui28Q1po)

## About Offsets
You must update the address in the Offset.h file, which is the address for the historical version. You can find the addresses in Physx from this [article of mine](https://super-timimus-838.notion.site/PhysX-6eccab27717c47d09a07917c4640e386?pvs=74).


## Technical Details

PUBG has a relatively large map model, and I reduce memory reads by calculating Transform and caching <ShapePtr, ActorPtr>. I read Shapes within a range into memory and use Embree for acceleration. Regarding destructible objects, they have some special properties (this may only exist in PUBG, as I am not a professional cheat program developer), after being destroyed, they become an object that still retains the same Shape address and Actor address, but the shape has changed or the Mesh type has changed. When the shape/type changes, the memory pointers in the Mesh also change. Therefore, I always need to judge whether their internal pointers have changed. This may be a unique property of PUBG, because GPT tells me that games usually need to rebuild the entire Mesh, but PUBG seems not to do so.


## Last
This project still has many bugs, and I will not fix them. It is sufficient for researchers and those who use it for their own purposes, but it is far from enough for those who hope to sell cheat software by copying the code.


## Please give me your star, thanks :)


## Cooperation
We are a professional software development company, involved in web development, security fields, desktop/mobile applications, and machine learning/LLM applications. If you need, please contact us via email (no illegal disturbances please) <jackjun0724@gmail.com>.
