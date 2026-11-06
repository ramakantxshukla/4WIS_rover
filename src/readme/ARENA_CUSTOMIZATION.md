# Arena Customization Guide

## Current Setup

The Arena.obj mesh is loaded in the Gazebo world at `/src/my_robot_bringup/worlds/acker.sdf`

## Adjusting Arena Appearance

### 1. Change Color

Edit `acker.sdf` and modify the material section:

```xml
<material>
  <ambient>R G B A</ambient>    <!-- Ambient lighting -->
  <diffuse>R G B A</diffuse>    <!-- Main color -->
  <specular>R G B A</specular>  <!-- Shine/reflection -->
  <emissive>R G B A</emissive>  <!-- Self-illumination -->
</material>
```

**Color values:** 0.0 to 1.0 for each channel (R, G, B, Alpha)

**Examples:**
```xml
<!-- Gray (current) -->
<ambient>0.7 0.7 0.7 1</ambient>
<diffuse>0.8 0.8 0.8 1</diffuse>

<!-- Brown/Tan -->
<ambient>0.6 0.5 0.4 1</ambient>
<diffuse>0.7 0.6 0.5 1</diffuse>

<!-- Blue -->
<ambient>0.2 0.3 0.6 1</ambient>
<diffuse>0.3 0.4 0.7 1</diffuse>

<!-- Green -->
<ambient>0.2 0.5 0.2 1</ambient>
<diffuse>0.3 0.6 0.3 1</diffuse>
```

### 2. Scale Arena

Change the `<scale>` parameter:

```xml
<mesh>
  <uri>file:///home/ram2/ackermann_steering_robot-master/src/my_robot_description/meshes/Arena.obj</uri>
  <scale>1.5 1.5 1.5</scale>  <!-- 50% larger -->
</mesh>
```

### 3. Rotate or Reposition

Change the `<pose>` parameter:

```xml
<model name='arena'>
  <pose>X Y Z Roll Pitch Yaw</pose>
  <!-- Example: Move 2m forward, rotate 45° -->
  <pose>2 0 0 0 0 0.785</pose>
</model>
```

### 4. Add Texture (If you have MTL file)

If Arena.obj has an associated Arena.mtl file:

```xml
<visual name='visual'>
  <geometry>
    <mesh>
      <uri>file:///path/to/Arena.obj</uri>
      <scale>1 1 1</scale>
    </mesh>
  </geometry>
  <!-- Material will be read from MTL file -->
</visual>
```

Gazebo will automatically look for Arena.mtl in the same directory.

### 5. Add Image Texture

If you have a texture image (e.g., Arena_texture.png):

```xml
<visual name='visual'>
  <geometry>
    <mesh>
      <uri>file:///path/to/Arena.obj</uri>
    </mesh>
  </geometry>
  <material>
    <ambient>1 1 1 1</ambient>
    <diffuse>1 1 1 1</diffuse>
    <pbr>
      <metal>
        <albedo_map>file:///path/to/Arena_texture.png</albedo_map>
      </metal>
    </pbr>
  </material>
</visual>
```

## Current Material Settings

```xml
<ambient>0.7 0.7 0.7 1</ambient>   <!-- Soft gray ambient light -->
<diffuse>0.8 0.8 0.8 1</diffuse>   <!-- Light gray surface -->
<specular>0.2 0.2 0.2 1</specular> <!-- Low shine -->
<emissive>0 0 0 1</emissive>       <!-- No self-glow -->
```

## Troubleshooting

### Arena appears pure white
- Check if material tags are inside `<visual>` section
- Verify ambient/diffuse values are set
- Check lighting in scene (sun intensity)

### Arena is too dark
- Increase diffuse values (0.9 0.9 0.9 1)
- Increase ambient values
- Add more lights to the scene

### Arena is too bright/shiny
- Reduce specular values (0.1 0.1 0.1 1)
- Reduce diffuse values

### Texture not showing
- Verify MTL file exists and is in same directory as OBJ
- Check texture image paths in MTL file are correct
- Ensure texture image formats are supported (PNG, JPG)

## Quick Color Presets

### Concrete Floor
```xml
<ambient>0.5 0.5 0.5 1</ambient>
<diffuse>0.6 0.6 0.6 1</diffuse>
<specular>0.1 0.1 0.1 1</specular>
```

### Wooden Floor
```xml
<ambient>0.4 0.3 0.2 1</ambient>
<diffuse>0.6 0.45 0.3 1</diffuse>
<specular>0.3 0.3 0.3 1</specular>
```

### Grass/Field
```xml
<ambient>0.2 0.4 0.2 1</ambient>
<diffuse>0.3 0.6 0.3 1</diffuse>
<specular>0.05 0.05 0.05 1</specular>
```

### Competition Mat (Blue)
```xml
<ambient>0.1 0.2 0.5 1</ambient>
<diffuse>0.2 0.3 0.7 1</diffuse>
<specular>0.2 0.2 0.2 1</specular>
```

## After Making Changes

```bash
cd ~/ackermann_steering_robot-master
colcon build --symlink-install
source install/setup.bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.py
```
