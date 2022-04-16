#include "flightros/fompl_planner/fompl_planner.hpp"

#define CONTROL_UPDATE_RATE 50.0

#define TINYPLY_IMPLEMENTATION

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace tinyply;

std::vector<FOMPL_Planner::float3> FOMPL_Planner::readPointCloud() {
  std::unique_ptr<std::istream> file_stream;
  std::vector<uint8_t> byte_buffer;
  std::string filepath =
    std::experimental::filesystem::path(__FILE__).parent_path().string() +
    "/data/point_cloud.ply";
  std::cout << filepath <<std::endl;
  try {
    file_stream.reset(new std::ifstream(filepath, std::ios::binary));

    if (!file_stream || file_stream->fail())
      throw std::runtime_error("file_stream failed to open " + filepath);

    file_stream->seekg(0, std::ios::end);
    const float size_mb = file_stream->tellg() * float(1e-6);
    file_stream->seekg(0, std::ios::beg);

    PlyFile file;
    file.parse_header(*file_stream);

    std::cout << "\t[ply_header] Type: "
              << (file.is_binary_file() ? "binary" : "ascii") << std::endl;
    for (const auto &c : file.get_comments())
      std::cout << "\t[ply_header] Comment: " << c << std::endl;
    for (const auto &c : file.get_info())
      std::cout << "\t[ply_header] Info: " << c << std::endl;

    for (const auto &e : file.get_elements()) {
      std::cout << "\t[ply_header] element: " << e.name << " (" << e.size << ")"
                << std::endl;
      for (const auto &p : e.properties) {
        std::cout << "\t[ply_header] \tproperty: " << p.name
                  << " (type=" << tinyply::PropertyTable[p.propertyType].str
                  << ")";
        if (p.isList)
          std::cout << " (list_type=" << tinyply::PropertyTable[p.listType].str
                    << ")";
        std::cout << std::endl;
      }
    }

    // Because most people have their own mesh types, tinyply treats parsed data
    // as structured/typed byte buffers. See examples below on how to marry your
    // own application-specific data structures with this one.
    std::shared_ptr<PlyData> vertices, normals, colors, texcoords, faces,
      tripstrip;

    // The header information can be used to programmatically extract properties
    // on elements known to exist in the header prior to reading the data. For
    // brevity of this sample, properties like vertex position are hard-coded:
    try {
      vertices =
        file.request_properties_from_element("vertex", {"x", "y", "z"});
    } catch (const std::exception &e) {
      std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    manual_timer read_timer;

    read_timer.start();
    file.read(*file_stream);
    read_timer.stop();

    const float parsing_time = read_timer.get() / 1000.f;
    std::cout << "\tparsing " << size_mb << "mb in " << parsing_time
              << " seconds [" << (size_mb / parsing_time) << " MBps]"
              << std::endl;

    if (vertices)
      std::cout << "\tRead " << vertices->count << " total vertices "
                << std::endl;


    const size_t numVerticesBytes = vertices->buffer.size_bytes();
    std::vector<float3> verts(vertices->count);
    std::memcpy(verts.data(), vertices->buffer.get(), numVerticesBytes);

    int idx = 0;
    for (const auto &point_tinyply : verts) {
      if (idx == 0) {
        points_ = Eigen::Vector3d(static_cast<double>(point_tinyply.x),
                                  static_cast<double>(point_tinyply.y),
                                  static_cast<double>(point_tinyply.z));
      } else {
        points_.conservativeResize(points_.rows(), points_.cols() + 1);
        points_.col(points_.cols() - 1) =
          Eigen::Vector3d(static_cast<double>(point_tinyply.x),
                          static_cast<double>(point_tinyply.y),
                          static_cast<double>(point_tinyply.z));
      }
      idx += 1;
    }
    kd_tree_.SetMatrixData(points_);

    return verts;
  } catch (const std::exception &e) {
    std::cerr << "Caught tinyply exception: " << e.what() << std::endl;
  }
  return verts;
}

void FOMPL_Planner::getBounds() {
  min_bounds = verts[0];
  max_bounds = verts[0];


  for (const auto &ver : verts) {
    if (ver.x < min_bounds.x) {
      min_bounds.x = ver.x;
    }

    if (ver.x > max_bounds.x) {
      max_bounds.x = ver.x;
    }

    if (ver.y < min_bounds.y) {
      min_bounds.y = ver.y;
    }

    if (ver.y > max_bounds.y) {
      max_bounds.y = ver.y;
    }

    if (ver.z > max_bounds.z) {
      max_bounds.z = ver.z;
    }
  }
}


void FOMPL_Planner::plan() {
  std::cout << "Planning.." << std::endl;
  // construct the state space we are planning in
  auto space(std::make_shared<ob::SE3StateSpace>());

  // set the bounds for the R^3 part of SE(3)
  ob::RealVectorBounds bounds(3);

  bounds.setLow(0, min_bounds.x);
  bounds.setLow(1, min_bounds.y);
  bounds.setLow(2, min_bounds.z);
  bounds.setHigh(0, max_bounds.x);
  bounds.setHigh(1, max_bounds.y);
  bounds.setHigh(2, max_bounds.z);
  std::cout << "Bounds min: x: " << min_bounds.x << "; y: " << min_bounds.y << "; z: "<<min_bounds.z<<"; " <<std::endl;
  std::cout << "Bounds max: x: " << max_bounds.x << "; y: " << max_bounds.y << "; z: "<<max_bounds.z<<"; " <<std::endl;
  space->setBounds(bounds);
  // define a simple setup class
  og::SimpleSetup ss(space);

  std::cout <<"Checkpoint 1" <<std::endl;
  // set state validity checking for this space
  ss.setStateValidityChecker(
    [](const ob::State *state) { return isStateValid(state); });

  // create a random start state
  ob::ScopedState<> start(space);
  // start->as<ob::RealVectorStateSpace::StateType>()->values[0] = current_position.x;
  // start->as<ob::RealVectorStateSpace::StateType>()->values[1] = current_position.y;
  // start->as<ob::RealVectorStateSpace::StateType>()->values[2] = current_position.z;
  start.random();
  std::cout <<"Checkpoint 2" <<std::endl;

  // create a random goal state
  ob::ScopedState<> goal(space);
  // goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_position.x;
  // goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_position.y;
  // goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal_position.z;
  goal.random();
  std::cout <<"Start point: " << start <<std::endl;
  std::cout <<"Goal point: " << goal <<std::endl;

  // set the start and goal states
  ss.setStartAndGoalStates(start, goal);
  std::cout <<"Checkpoint 3" <<std::endl;

  // this call is optional, but we put it in to get more output information
  ss.setup();
  ss.print();
  std::cout <<"Checkpoint 4" <<std::endl;

  // attempt to solve the problem within one second of planning time
  ob::PlannerStatus solved = ss.solve(1.0);
  std::cout <<"Checkpoint 5" <<std::endl;

  if (solved) {
    std::cout << "Found solution:" << std::endl;

    ob::PlannerData pd(ss.getSpaceInformation());
    ss.getPlannerData(pd);
    /** backup cout buffer and redirect to path.txt **/
    std::ofstream out0(
      std::experimental::filesystem::path(__FILE__).parent_path().string() +
      "/data//vertices.txt");
    auto *coutbuf0 = std::cout.rdbuf();
    std::cout.rdbuf(out0.rdbuf());


    unsigned int num_vertices = pd.numVertices();
    for (unsigned int i = 0; i < num_vertices; i++) {
      Eigen::Vector3d p = stateToEigen(pd.getVertex(i).getState());
      std::cout << p.x() << " " << p.y() << " " << p.z() << " " << std::endl;
    }

    /** reset cout buffer **/
    std::cout.rdbuf(coutbuf0);

    /** backup cout buffer and redirect to path.txt **/
    std::ofstream out00(
      std::experimental::filesystem::path(__FILE__).parent_path().string() +
      "/data/edges.txt");
    auto *coutbuf00 = std::cout.rdbuf();
    std::cout.rdbuf(out00.rdbuf());

    for (unsigned int i = 0; i < num_vertices; i++) {
      std::vector<unsigned int> e;
      pd.getEdges(i, e);
      for (const auto &j : e) {
        std::cout << i << " " << j << " " << std::endl;
      }
    }

    /** reset cout buffer **/
    std::cout.rdbuf(coutbuf00);

    // save solution in solution_path.txt
    /** backup cout buffer and redirect to path.txt **/
    ss.simplifySolution();
    std::ofstream out(
      std::experimental::filesystem::path(__FILE__).parent_path().string() +
      "/data/solution_path.txt");
    auto *coutbuf = std::cout.rdbuf();
    std::cout.rdbuf(out.rdbuf());
    ss.getSolutionPath().printAsMatrix(std::cout);
    /** reset cout buffer **/
    std::cout.rdbuf(coutbuf);
    ss.getSolutionPath().print(std::cout);
    path_.clear();
    path_ = ss.getSolutionPath().getStates();
    vecs_.clear();
    for (auto const &pos : path_) {
      vecs_.push_back(stateToEigen(pos));
    }

    if (path_.size() > 1) {
      solution_found = true;
    }
  } else
    std::cout << "No solution found" << std::endl;
}

Eigen::Vector3d FOMPL_Planner::stateToEigen(const ompl::base::State *state) {
  // cast the abstract state type to the type we expect
  const auto *se3state = state->as<ob::SE3StateSpace::StateType>();

  // extract the first component of the state and cast it to what we expect
  const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

  // extract the second component of the state and cast it to what we expect
  //  const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

  // check validity of state defined by pos & rot
  float x = pos->values[0];
  float y = pos->values[1];
  float z = pos->values[2];

  // return a value that is always true but uses the two variables we define, so
  // we avoid compiler warnings
  // return isInRange(x, y, z);
  Eigen::Vector3d query_pos{x, y, z};
  return query_pos;
}

void FOMPL_Planner::convertOMPLPathToMsg() {
  //Reference &msg -> just another name for last_traj_msg
    mav_planning_msgs::PolynomialTrajectory4D &msg = path_msg_;
    msg.segments.clear();

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world"; // "odom"

    std::vector<ompl::base::State *> &states = path_;
    size_t N = states.size();
    for (size_t i = 0; i<N; i++) {
        ompl::base::State *p_s = states[i];
        const double &x_s = p_s->as<ob::RealVectorStateSpace::StateType>()->values[0];
        const double &y_s = p_s->as<ob::RealVectorStateSpace::StateType>()->values[1];
        const double &z_s = p_s->as<ob::RealVectorStateSpace::StateType>()->values[2];
        // double z_s = 0.;
        double yaw_s = 0.;
        ROS_INFO_STREAM("states["<< i <<"], x_s: " << x_s << "; y_s: " << y_s << "; z_s: " << z_s);

        mav_planning_msgs::PolynomialSegment4D segment;
        segment.header = msg.header;
        segment.num_coeffs = 0;
        segment.segment_time = ros::Duration(0.);
        
        segment.x.push_back(x_s);
        segment.y.push_back(y_s);
        segment.z.push_back(z_s);
        segment.yaw.push_back(yaw_s);
        msg.segments.push_back(segment);
    }
}

bool FOMPL_Planner::isStateValid(const ob::State *state) {
  // cast the abstract state type to the type we expect
  const auto *se3state = state->as<ob::SE3StateSpace::StateType>();

  // extract the first component of the state and cast it to what we expect
  const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

  // extract the second component of the state and cast it to what we expect
  //  const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

  // check validity of state defined by pos & rot
  float x = pos->values[0];
  float y = pos->values[1];
  float z = pos->values[2];
  // return a value that is always true but uses the two variables we define, so
  // we avoid compiler warnings
  // return isInRange(x, y, z);
  Eigen::Vector3d query_pos{x, y, z};
  return searchRadius(query_pos, range);
}

bool FOMPL_Planner::isInRange(float x, float y, float z) {
  bool validState = true;
  bool outOfBound = false;

  for (const auto &ver : verts) {
    // check if box around quad is occupied
    if (abs(ver.z - z) <= range) {
      if (abs(ver.x - x) <= range) {
        if (abs(ver.y - y) <= range) {
          validState = false;
        }
      }
    } else {
      if (ver.z - z >= 0) {
        outOfBound = true;
      }
    }


    if (outOfBound || !validState) {
      break;
    }
  }
  return validState;
}


bool FOMPL_Planner::searchRadius(const Eigen::Vector3d &query_point,
                                   const double radius) {
  std::vector<int> indices;
  std::vector<double> distances_squared;
  kd_tree_.SearchRadius(query_point, radius, indices, distances_squared);

  if (indices.size() == 0) {
    return true;
  }

  for (const auto &close_point_idx : indices) {
    // get point, check if within drone body
    Eigen::Vector3d close_point = points_.col(close_point_idx);
    // project point on each body axis and check distance
    Eigen::Vector3d close_point_body = (close_point - query_point);
    if (std::abs(close_point_body.x()) <= range &&
        std::abs(close_point_body.y()) <= range &&
        std::abs(close_point_body.z()) <= range) {
      // point is in collision
      return false;
    }
  }

  return true;
}

void FOMPL_Planner::executePath() {
  // initialization
  assert(!vecs_.empty());
  assert(vecs_.at(0).x() != NULL);

  // compute trajectory
  std::size_t num_waypoints = vecs_.size();
  int scaling = 10;

  std::vector<Eigen::Vector3d> way_points;

  for (int i = 0; i < int(num_waypoints - 1); i++) {
    Eigen::Vector3d diff_vec = vecs_.at(i + 1) - vecs_.at(i);
    double norm = diff_vec.norm();
    for (int j = 0; j < int(norm * scaling); j++) {
      way_points.push_back(vecs_.at(i) + j * (diff_vec / (norm * scaling)));
    }
  }

  // Flightmare
  Vector<3> B_r_BC(0.0, 0.0, 0.3);
  Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  std::cout << R_BC << std::endl;
  rgb_camera_->setFOV(90);
  rgb_camera_->setWidth(720);
  rgb_camera_->setHeight(480);
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  quad_ptr_->addRGBCamera(rgb_camera_);

  quad_state_.setZero();
  quad_state_.x[QS::POSX] = (Scalar)vecs_.at(0).x();
  quad_state_.x[QS::POSY] = (Scalar)vecs_.at(0).y();
  quad_state_.x[QS::POSZ] = (Scalar)vecs_.at(0).z();

  quad_ptr_->reset(quad_state_);

  // connect unity
  setUnity(unity_render_);
  connectUnity();
  assert(unity_ready_);
  assert(unity_render_);

  int counter = 0;

  while (unity_render_ && unity_ready_) {
    quad_state_.x[QS::POSX] = (Scalar)way_points.at(counter).x();
    quad_state_.x[QS::POSY] = (Scalar)way_points.at(counter).y();
    quad_state_.x[QS::POSZ] = (Scalar)way_points.at(counter).z();
    quad_state_.x[QS::ATTW] = (Scalar)0;
    quad_state_.x[QS::ATTX] = (Scalar)0;
    quad_state_.x[QS::ATTY] = (Scalar)0;
    quad_state_.x[QS::ATTZ] = (Scalar)0;

    quad_ptr_->setState(quad_state_);

    unity_bridge_ptr_->getRender(0);
    unity_bridge_ptr_->handleOutput();

    counter += 1;
    counter = counter % way_points.size();
  }
}

void FOMPL_Planner::visualizeTrajectory() {
  std::cout <<"visualiing trajectory..." << std::endl;
  visualization_msgs::Marker::Ptr p_traj_points, p_traj_edges;
  p_traj_points = visualization_msgs::Marker::Ptr( new visualization_msgs::Marker() );
  p_traj_edges = visualization_msgs::Marker::Ptr( new visualization_msgs::Marker() );
  visualization_msgs::Marker &traj_points = *p_traj_points;
  visualization_msgs::Marker &traj_edges = *p_traj_edges;
  traj_points.header.frame_id = traj_edges.header.frame_id = "world"; // "odom"
  traj_points.header.stamp = traj_edges.header.stamp = ros::Time::now();
  traj_points.ns = traj_edges.ns = "planned_trajectory";
  traj_points.action = traj_edges.action = visualization_msgs::Marker::ADD;
  traj_points.pose.orientation.w = traj_edges.pose.orientation.w = 1.0;
  traj_points.id = 0;
  traj_edges.id = 1;

  traj_points.type = visualization_msgs::Marker::POINTS;
  traj_edges.type = visualization_msgs::Marker::LINE_STRIP;

  // POINTS markers use x and y scale for width/height respectively
  traj_points.scale.x = 0.05;
  traj_points.scale.y = 0.05;
  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  traj_edges.scale.x = 0.025;
  traj_points.color.r = 1.00f;
  traj_points.color.g = 0.65f;
  traj_points.color.b = 0.00f;
  traj_edges.color.r = 1.00f;
  traj_edges.color.g = 0.65f;
  traj_edges.color.b = 0.00f;
  
  traj_points.color.a = 1.0;
  traj_edges.color.a = 0.75;

  // *** Conversion to Rviz markers message ***
  // Create the vertices for the points and lines
  const mav_planning_msgs::PolynomialTrajectory4D &msg = path_msg_;
  size_t N = msg.segments.size();
  for (size_t i=0; i<N; i++)
  {
      const mav_planning_msgs::PolynomialSegment4D &segment = msg.segments[i];
      std::cout <<"Segment: x: " << segment.x[0] << "; y: " <<segment.y[0] <<"; z: " <<segment.z[0]<<"; " <<std::endl;
      geometry_msgs::Point p;
      p.x = segment.x[0];
      p.y = segment.y[0];
      p.z = segment.z[0];
      traj_points.points.push_back(p);
      traj_edges.points.push_back(p);
  } 

  //case WHITE:
  trajectory_markers_pub.publish(traj_points);
  trajectory_markers_pub.publish(traj_edges);
}
bool FOMPL_Planner::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    std::cout << "Unity Bridge is created." << std::endl;
  }
  return true;
}

bool FOMPL_Planner::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

void FOMPL_Planner::currentPositionCallback(const geometry_msgs::Point::ConstPtr& p_msg) {
    current_position = *p_msg;
    ROS_INFO_STREAM("New current position, x: " << current_position.x << "; y: " << current_position.y << "; z: " << current_position.z);
}

void FOMPL_Planner::goalPositionCallback(const geometry_msgs::Point::ConstPtr& p_msg) {
    goal_position = *p_msg;
    ROS_INFO_STREAM("New goal position, x: " << goal_position.x << "; y: " << goal_position.y << "; z: " << goal_position.z);

    std::cout << "Plan & stuff" << std::endl;
    while (!FOMPL_Planner::solution_found) {
      FOMPL_Planner::plan();  
      FOMPL_Planner::convertOMPLPathToMsg();
      FOMPL_Planner::trajectory_pub.publish(path_msg_);
      FOMPL_Planner::visualizeTrajectory();
    }

    std::cout << "Execute" << std::endl;
    FOMPL_Planner::executePath();

}


int main(int argc, char *argv[]) {
  // initialize ROS
  ros::init(argc, argv, "flightmare_example");
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");

  // ROS topics
  FOMPL_Planner::current_position_sub = pnh.subscribe("current_position", 10, FOMPL_Planner::currentPositionCallback);
  FOMPL_Planner::goal_position_sub = pnh.subscribe("goal_position", 10, FOMPL_Planner::goalPositionCallback);
    //trajectory_pub_raw = pnode_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("planned_trajectory_raw", 1);
  FOMPL_Planner::trajectory_pub = pnh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("planned_trajectory", 1);
  FOMPL_Planner::trajectory_markers_pub = pnh.advertise<visualization_msgs::Marker>("planned_path_markers", 1);
  // quad initialization
  FOMPL_Planner::quad_ptr_ = std::make_unique<Quadrotor>();
  // add camera
  FOMPL_Planner::rgb_camera_ = std::make_unique<RGBCamera>();

  std::cout << "Read PointCloud" << std::endl;
  FOMPL_Planner::verts = FOMPL_Planner::readPointCloud();

  std::cout << "Get Bounds" << std::endl;
  FOMPL_Planner::getBounds();

  ros::spin();

  return 0;
}

