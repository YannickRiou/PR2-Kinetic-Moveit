/*
 * This file was generated by generate_typesystem.py.
 * Filename:    scene_types
 * Name:        scene
 * Description: No description given
 * Version:     1.0
 * Vendor:      None
 */
#ifndef __SCENE_TYPES_H__
#define __SCENE_TYPES_H__

#include <rs/feature_structure_proxy.h>
#include <rs/types/type_definitions.h>
#include <rs/types/core_types.h>
#include <rs/types/pcl_types.h>
#include <rs/types/cv_types.h>

namespace rs
{

/*
 * No description given
 */
class Cluster : public Identifiable
{
private:
  void initFields()
  {
    annotations.init(this, "annotations");
    points.init(this, "points");
    centroid.init(this, "centroid");
    rois.init(this, "rois");
    source.init(this, "source");
  }
public:
  // No description given
  ListFeatureStructureEntry<Annotation> annotations;
  // No description given
  ComplexFeatureStructureEntry<ClusterPoints> points;
  // Centroid of a cluster
  ArrayFeatureStructureEntry<float> centroid;
  // No description given
  ComplexFeatureStructureEntry<ImageROI> rois;
  // No description given
  FeatureStructureEntry<std::string> source;

  Cluster(const Cluster &other) :
      Identifiable(other)
  {
    initFields();
  }

  Cluster(uima::FeatureStructure fs) :
      Identifiable(fs)
  {
    initFields();
  }
};

/*
 * No description given
 */
class MergedCluster : public Cluster
{
private:
  void initFields()
  {
  }
public:

  MergedCluster(const MergedCluster &other) :
      Cluster(other)
  {
    initFields();
  }

  MergedCluster(uima::FeatureStructure fs) :
      Cluster(fs)
  {
    initFields();
  }
};

/*
 * No description given
 */
class Object : public Cluster
{
private:
  void initFields()
  {
    uid.init(this, "uid");
    lastSeen.init(this, "lastSeen");
    wasSeen.init(this, "wasSeen");
    inView.init(this, "inView");
    disappeared.init(this, "disappeared");
    clusters.init(this, "clusters");
  }
public:
  // No description given
  FeatureStructureEntry<std::string> uid;
  // No description given
  FeatureStructureEntry<INT64> lastSeen;
  // True if the object was seen in the last frame
  FeatureStructureEntry<bool> wasSeen;
  // True if the object should be in camera View
  FeatureStructureEntry<bool> inView;
  // True if the object was not seen while it should have been in view
  FeatureStructureEntry<bool> disappeared;
  // No description given
  ArrayFeatureStructureEntry<std::string> clusters;

  Object(const Object &other) :
      Cluster(other)
  {
    initFields();
  }

  Object(uima::FeatureStructure fs) :
      Cluster(fs)
  {
    initFields();
  }
};

}

TYPE_TRAIT(rs::Cluster, RS_SCENE_CLUSTER)
TYPE_TRAIT(rs::MergedCluster, RS_SCENE_MERGEDCLUSTER)
TYPE_TRAIT(rs::Object, RS_SCENE_OBJECT)

#endif /* __SCENE_TYPES_H__ */