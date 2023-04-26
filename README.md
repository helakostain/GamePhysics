# Game physics
To use this project for physics simulation using PhysX physics engine you need to have these libraries:

 - Assimp https://github.com/assimp/assimp
 - Glew 2.1.0 https://glew.sourceforge.net/
 - GLFW 3.3.8 https://www.glfw.org/
 - GLM https://github.com/g-truc/glm
 - PhysX 4.1 https://github.com/NVIDIAGameWorks/PhysX
 - SOIL https://github.com/Pikachuxxxx/SOIL
 - spdlog https://github.com/gabime/spdlog
 - Visual Studio 2022
Currently this project is configured for running in "Release" mode in visual studio, but it is possible with changes in libraries to be run as "Debug". Debug mode is much slower, but offers to great debugging options as mesh validation and using the PhysX Visual Debugger program for more debugging options. Also in the console is written the performance of the application on system and in the title name of the window is also shown current Frame per Second. 

# Setup

All libraries need to be compiled and put in the same parent folder where the "GamePhysics" project is and named "Knihovny", then it should work directly. For PhysX and Visual Studio 2022 there need to be implemented small changes listed here: https://github.com/NVIDIAGameWorks/PhysX/pull/577/files. For MD and MDd run there also need to be the library flag changed, which is explained here: https://github.com/NVIDIAGameWorks/PhysX/issues/115 and https://github.com/NVIDIAGameWorks/PhysX/issues/135.

## Cuda accelaration
If you want for rigid bodies to be accelerated on GPU instead of CPU there need to be changed the flag in Scene.hpp file. The flag is called "cudaON" and needs to be set to true. 

## Models used
Models can be changed simply. Any new model needs to be put to the folder listed in this path "GamePhysics/GamePhysics/Models". Each model needs to be type of Wavefront OBJ (it contains two files, one with .obj and second ending with .mtl). If the model does not have a texture file and uses the material for colours, then the default texture "white_tex.png" is used. If the model has a texture, then the texture needs to be in the folder in this path "GamePhysics/GamePhysics/Textures". The texture needs to be in this image format: BMP, PNG, JPG, TGA, DDS, PSD, HDR. More details about support of images for SOIL can be found in this repository: https://github.com/Pikachuxxxx/SOIL. 
When the model is in correct directory, then it can be loaded in the Scene.cpp in constructor of this class after skybox model and before camera is added. The code is following: 
```this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get(model_name), ShaderInstances::phong(), TextureManager::getOrEmplace("model_name", "Textures/white_tex.png"), drawable_object.size(), true, actor_type));```.
Parameter "model_name" is the name of the model, so if we have model named example.obj with material in example.mtl, then the "model_name" is gonna be "example". Default for light is Phong reflection model, but it is possible to change it to: constant, phong_no_textures and phong_norm. I recommend to use the default setting for best visualization. If it is desired to use different texture then the "white_tex.png" needs to be changed to the name of the image file with texture like "example.png". "actor_type" is for what type of the actor in the physics is this model gonna be. This parameter is type of int, and number 0 is for static actor, number 1 for dynamic collisionable actor and number 2 is for kinematic actor (which is currently also the character controller). 
## Lightning changes
Lightning can be changed easily. There is multiple light modes: ambient light, point light, directional light, spotlight. Ambient light is already activated and I recommend to leave it like this. Point light can be added with this methos in Scene.cpp constructor bellow models inicialization ```emplaceLight(const glm::vec3 color, const glm::vec3 pos, const gl::Light type)```.  The light type would be "gl::Light::Point". Directional light has example in the constructor. Spotlight is also already implemented as a flashlight which is activated by key "E" during simulation. 
## Game controls
The game is controlled by keyboard keys WASD and arrow keys for movement similar to many games. Looking through camera is done through mouse, where left button of the mouse needs to be pushed for camera movement. Character can jump using Space key. Key F is for shooting collisionable balls. Game is ended with closing the window. 



