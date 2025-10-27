# obstacles_polygon_detection

> Criar obstáculos com base em marca fiduciária

Pacote ROS 2 que receba detecções de marcas fiduciárias e as converta em obstaculos a serem publicados numa mensagem personalizada baseada no tipo [Polygon](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Polygon.html) só que implementando como um array.

Considerando o uso de Aruco, o nó deve subscrever `/aruco_markers` (`ros2_aruco_interfaces.msg.ArucoMarkers`)

Sugestão de implementação:
- Definir a posição global do Aruco (podem utilizar a própria calibração)
- Definir um arquivo YAML com os grupos de marcas (exemplo a seguir)
```yml
obstacles:
  - name: obst1
    ids: [1, 2, 3]
  - name: obst2
    ids: [4, 5, 6, 7]
```
- Junta as posições globais dos arucos em um grupo e um polígono na mensagem criada
- Publica a mensagem ja preparando alguma visualização no Rviz

## Observações
- A escolha do tipo de marca é de cargo do executante (Aruco, AprilTag, etc)
- A escolha entre Python e C++ também é subjetiva