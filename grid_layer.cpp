#include "grid_layer.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::GridLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace simple_layer_namespace
{

GridLayer::GridLayer() {}

void GridLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &GridLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void GridLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}


void GridLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void GridLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  double mark_x = robot_x + cos(robot_yaw), mark_y = robot_y + sin(robot_yaw);
  unsigned int mx;
  unsigned int my;
  if(worldToMap(mark_x, mark_y, mx, my)){
    setCost(mx, my, LETHAL_OBSTACLE);
  }
  
  *min_x = std::min(*min_x, mark_x);
  *min_y = std::min(*min_y, mark_y);
  *max_x = std::max(*max_x, mark_x);
  *max_y = std::max(*max_y, mark_y);
}



void GridLayer::msgSub(const geometry_msgs::Polygon::ConstPtr& msg){
	int size=msg->points.size();
	xs.clear();
	ys.clear();
	for(int i=0; i<size; i++){
		xs.push_back(msg->points[i].x);
		ys.push_back(msg->points[i].y);
	}
};

void GridLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
	int numberOfDots=xs.size();
	unsigned int minx=99999, miny=99999, maxx=0, maxy=0;
	unsigned int mapMinx, mapMiny, mapMaxx, mapMaxy;
	for(int k=0; k<numberOfDots; k++){
		unsigned int mx, my;
		if(xs[k]>maxx) maxx=xs[k];
		if(xs[k]<minx) minx=xs[k];
		if(ys[k]>maxy) maxy=ys[k];
		if(ys[k]<miny) miny=ys[k];
	}
	worldToMap(minx, miny, mapMinx, mapMiny);
	worldToMap(maxx, maxy, mapMaxx, mapMaxy);
	for (int j = mapMiny; j < mapMaxy; j++){
		for (int i = mapMinx; i < mapMaxx; i++){
			int control=0;
			for(int k=0; k<numberOfDots; k++){
				unsigned int lx, ly, ox, oy, rx, ry;
				worldToMap(xs[k], ys[k], ox, oy);
				if(k==0) worldToMap(xs[numberOfDots-1], ys[numberOfDots-1], lx, ly);
				else worldToMap(xs[k-1], ys[k-1], lx, ly);
				if(k==numberOfDots-1) worldToMap(xs[0], ys[0], rx, ry);
				else worldToMap(xs[k+1], ys[k+1], rx, ry);
				int LvX = (int)lx-(int)ox, LvY = (int)ly-(int)oy, RvX = (int)rx-(int)ox, RvY = (int)ry-(int)oy;
				if(RvX*LvY-RvY*LvX >=0){
					int actX = (int) i - (int) ox, actY = (int) j - (int) oy;
					if(RvX*actY - RvY*actX >=0 && LvX*actY - LvY*actX <=0) control++;
					else continue;
				}
				else{
					std::cout << "CONCAVITY HERE! " << std::endl;
				}
			}
			if(control==numberOfDots){	
				int index = getIndex(i, j);
				if (master_grid.getCost(i, j) >= 100)
							continue;
				master_grid.setCost(i, j, LETHAL_OBSTACLE);
			}
		}
	}
}

} // end namespace
