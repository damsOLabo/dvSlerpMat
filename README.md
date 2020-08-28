| Plug-ins | maya |
|:--------:|:--------:|
|Mode rig|True|

# dvSlerpMat
![scene|medium](./docs/images/scene.png)

## Presentation
TODO

## Attributs

```bash
inputs: {
    rigMode
    matrixA
    matrixB
    parentInverseMatrix
}

storage: {
    offsetMatrix
}

outputs: {
    xform
        |-translate
        |    |-translateX  
        |    |-translateY  
        |    |-translateZ  
        |-rotate
        |    |-rotateX  
        |    |-rotateY  
        |    |-rotateZ  
        |-scale
        |    |-scaleX  
        |    |-scaleY  
        |    |-scaleZ

}
```
## Node editor Connection
![scene_node](./docs/images/scene_node.png "scene_node")

![outliner](./docs/images/outliner.png "outliner")

## Commands (python)
