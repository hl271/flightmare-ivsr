# How to export trajectory of type `std::vector<Eigen::Vector3D>` to RViz markers
## Create traj_points and traj_edges
```
visualization_msgs::Marker traj_points;
visualization_msgs::Marker traj_edges;

traj_points.type = visualization_msgs::Marker::POINTS;
traj_points.type = visualization_msgs::Marker::LINE_STRIPS;

```

traj_points.points, traj_edges.points =>  geometry_msgs