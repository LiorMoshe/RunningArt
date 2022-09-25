# RunningArt

Draw out running routes replicating pen drawings in your home town.
The repository contains a flask backend which responds to requests from
the [RunningArt gui](https://github.com/LiorMoshe/RunningArtDemoGui).

Behind the scenes the map given from openstreetmap is converted into a graph
of lines for which we can approximate the requested shape:
![Conversion of the map to a line graph](https://github.com/LiorMoshe/RunningArt/blob/master/resources/images/lines_map.png)

Once we have the input line graph and the input line graph (representing the requested drawing) we perform a weighted variant of the djikstra algorithm to search for the best fit subgraph to the input graph.
Currently works for english characters:
![F Approximation](https://github.com/LiorMoshe/RunningArt/blob/master/resources/images/f_example.png)
![E Approximation](https://github.com/LiorMoshe/RunningArt/blob/master/resources/images/e_example.png)
![L Approximation](https://github.com/LiorMoshe/RunningArt/blob/master/resources/images/l_example.png)
![H Approximation](https://github.com/LiorMoshe/RunningArt/blob/master/resources/images/h_example.png)
