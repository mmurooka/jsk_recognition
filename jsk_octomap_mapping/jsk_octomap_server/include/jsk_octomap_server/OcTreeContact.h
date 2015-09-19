#ifndef OCTOMAP_OCTREECONTACT_H
#define OCTOMAP_OCTREECONTACT_H


#include <octomap/OcTree.h>

namespace octomap {

  /**
   * octomap main map data structure, stores 3D occupancy grid map in an OcTree.
   * Basic functionality is implemented in OcTreeBase.
   *
   */
  class OcTreeContact : public OcTree {

  public:
    inline bool isNodeFree(const OcTreeNode* occupancyNode) const{
      return (occupancyNode->getLogOdds() <= this->free_prob_thres_log);
    }
    inline bool isNodeFree(const OcTreeNode& occupancyNode) const{
      return (occupancyNode.getLogOdds() <= this->free_prob_thres_log);
    }
    inline bool isNodeUnknown(const OcTreeNode* occupancyNode) const{
      return (!isNodeOccupied(occupancyNode) and !isNodeFree(occupancyNode));
    }
    inline bool isNodeUnknown(const OcTreeNode& occupancyNode) const{
      return (!isNodeOccupied(occupancyNode) and !isNodeFree(occupancyNode));
    }

    void setFreeThres(double prob){free_prob_thres_log = logodds(prob); }
    void setProbHitContactSensor(double prob){prob_hit_contact_sensor_log = logodds(prob);}
    void setProbMissContactSensor(double prob){prob_miss_contact_sensor_log = logodds(prob);}

    double getFreeThres() const {return probability(free_prob_thres_log); }
    float getFreeThresLog() const {return free_prob_thres_log; }

    double getProbHitContactSensor() const {return probability(prob_hit_contact_sensor_log); }
    float getProbHitContactSensorLog() const {return prob_hit_contact_sensor_log; }
    double getProbMissContactSensor() const {return probability(prob_miss_contact_sensor_log); }
    float getProbMissContactSensorLog() const {return prob_miss_contact_sensor_log; }

    float prob_hit_contact_sensor_log;
    float prob_miss_contact_sensor_log;
    float free_prob_thres_log;

    OcTreeContact(double resolution) : OcTree(resolution)
    {
      setOccupancyThres(0.7);   // = 0.0 in logodds
      setFreeThres(0.3);   // = 0.0 in logodds
      setProbMiss(0.2);         // = -0.4 in logodds
      setProbHitContactSensor(0.7);
      setProbMissContactSensor(0.1);
    };
  };

} // end namespace

#endif
