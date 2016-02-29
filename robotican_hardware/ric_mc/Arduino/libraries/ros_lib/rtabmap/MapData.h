#ifndef _ROS_rtabmap_MapData_h
#define _ROS_rtabmap_MapData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "rtabmap/Bytes.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Pose.h"

namespace rtabmap
{

  class MapData : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t mapIDs_length;
      int32_t st_mapIDs;
      int32_t * mapIDs;
      uint8_t maps_length;
      int32_t st_maps;
      int32_t * maps;
      uint8_t imageIDs_length;
      int32_t st_imageIDs;
      int32_t * imageIDs;
      uint8_t images_length;
      rtabmap::Bytes st_images;
      rtabmap::Bytes * images;
      uint8_t depthIDs_length;
      int32_t st_depthIDs;
      int32_t * depthIDs;
      uint8_t depths_length;
      rtabmap::Bytes st_depths;
      rtabmap::Bytes * depths;
      uint8_t depth2DIDs_length;
      int32_t st_depth2DIDs;
      int32_t * depth2DIDs;
      uint8_t depth2Ds_length;
      rtabmap::Bytes st_depth2Ds;
      rtabmap::Bytes * depth2Ds;
      uint8_t depthConstantIDs_length;
      int32_t st_depthConstantIDs;
      int32_t * depthConstantIDs;
      uint8_t depthConstants_length;
      float st_depthConstants;
      float * depthConstants;
      uint8_t localTransformIDs_length;
      int32_t st_localTransformIDs;
      int32_t * localTransformIDs;
      uint8_t localTransforms_length;
      geometry_msgs::Transform st_localTransforms;
      geometry_msgs::Transform * localTransforms;
      uint8_t poseIDs_length;
      int32_t st_poseIDs;
      int32_t * poseIDs;
      uint8_t poses_length;
      geometry_msgs::Pose st_poses;
      geometry_msgs::Pose * poses;
      uint8_t constraintFromIDs_length;
      int32_t st_constraintFromIDs;
      int32_t * constraintFromIDs;
      uint8_t constraintToIDs_length;
      int32_t st_constraintToIDs;
      int32_t * constraintToIDs;
      uint8_t constraintTypes_length;
      int32_t st_constraintTypes;
      int32_t * constraintTypes;
      uint8_t constraints_length;
      geometry_msgs::Transform st_constraints;
      geometry_msgs::Transform * constraints;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = mapIDs_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < mapIDs_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_mapIDsi;
      u_mapIDsi.real = this->mapIDs[i];
      *(outbuffer + offset + 0) = (u_mapIDsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mapIDsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mapIDsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mapIDsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mapIDs[i]);
      }
      *(outbuffer + offset++) = maps_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < maps_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_mapsi;
      u_mapsi.real = this->maps[i];
      *(outbuffer + offset + 0) = (u_mapsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mapsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mapsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mapsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->maps[i]);
      }
      *(outbuffer + offset++) = imageIDs_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < imageIDs_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_imageIDsi;
      u_imageIDsi.real = this->imageIDs[i];
      *(outbuffer + offset + 0) = (u_imageIDsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_imageIDsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_imageIDsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_imageIDsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->imageIDs[i]);
      }
      *(outbuffer + offset++) = images_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < images_length; i++){
      offset += this->images[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = depthIDs_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < depthIDs_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_depthIDsi;
      u_depthIDsi.real = this->depthIDs[i];
      *(outbuffer + offset + 0) = (u_depthIDsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_depthIDsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_depthIDsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_depthIDsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->depthIDs[i]);
      }
      *(outbuffer + offset++) = depths_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < depths_length; i++){
      offset += this->depths[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = depth2DIDs_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < depth2DIDs_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_depth2DIDsi;
      u_depth2DIDsi.real = this->depth2DIDs[i];
      *(outbuffer + offset + 0) = (u_depth2DIDsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_depth2DIDsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_depth2DIDsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_depth2DIDsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->depth2DIDs[i]);
      }
      *(outbuffer + offset++) = depth2Ds_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < depth2Ds_length; i++){
      offset += this->depth2Ds[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = depthConstantIDs_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < depthConstantIDs_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_depthConstantIDsi;
      u_depthConstantIDsi.real = this->depthConstantIDs[i];
      *(outbuffer + offset + 0) = (u_depthConstantIDsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_depthConstantIDsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_depthConstantIDsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_depthConstantIDsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->depthConstantIDs[i]);
      }
      *(outbuffer + offset++) = depthConstants_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < depthConstants_length; i++){
      union {
        float real;
        uint32_t base;
      } u_depthConstantsi;
      u_depthConstantsi.real = this->depthConstants[i];
      *(outbuffer + offset + 0) = (u_depthConstantsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_depthConstantsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_depthConstantsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_depthConstantsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->depthConstants[i]);
      }
      *(outbuffer + offset++) = localTransformIDs_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < localTransformIDs_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_localTransformIDsi;
      u_localTransformIDsi.real = this->localTransformIDs[i];
      *(outbuffer + offset + 0) = (u_localTransformIDsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_localTransformIDsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_localTransformIDsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_localTransformIDsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->localTransformIDs[i]);
      }
      *(outbuffer + offset++) = localTransforms_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < localTransforms_length; i++){
      offset += this->localTransforms[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = poseIDs_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < poseIDs_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_poseIDsi;
      u_poseIDsi.real = this->poseIDs[i];
      *(outbuffer + offset + 0) = (u_poseIDsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_poseIDsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_poseIDsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_poseIDsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->poseIDs[i]);
      }
      *(outbuffer + offset++) = poses_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < poses_length; i++){
      offset += this->poses[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = constraintFromIDs_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < constraintFromIDs_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_constraintFromIDsi;
      u_constraintFromIDsi.real = this->constraintFromIDs[i];
      *(outbuffer + offset + 0) = (u_constraintFromIDsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_constraintFromIDsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_constraintFromIDsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_constraintFromIDsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->constraintFromIDs[i]);
      }
      *(outbuffer + offset++) = constraintToIDs_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < constraintToIDs_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_constraintToIDsi;
      u_constraintToIDsi.real = this->constraintToIDs[i];
      *(outbuffer + offset + 0) = (u_constraintToIDsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_constraintToIDsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_constraintToIDsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_constraintToIDsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->constraintToIDs[i]);
      }
      *(outbuffer + offset++) = constraintTypes_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < constraintTypes_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_constraintTypesi;
      u_constraintTypesi.real = this->constraintTypes[i];
      *(outbuffer + offset + 0) = (u_constraintTypesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_constraintTypesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_constraintTypesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_constraintTypesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->constraintTypes[i]);
      }
      *(outbuffer + offset++) = constraints_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < constraints_length; i++){
      offset += this->constraints[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t mapIDs_lengthT = *(inbuffer + offset++);
      if(mapIDs_lengthT > mapIDs_length)
        this->mapIDs = (int32_t*)realloc(this->mapIDs, mapIDs_lengthT * sizeof(int32_t));
      offset += 3;
      mapIDs_length = mapIDs_lengthT;
      for( uint8_t i = 0; i < mapIDs_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_mapIDs;
      u_st_mapIDs.base = 0;
      u_st_mapIDs.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_mapIDs.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_mapIDs.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_mapIDs.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_mapIDs = u_st_mapIDs.real;
      offset += sizeof(this->st_mapIDs);
        memcpy( &(this->mapIDs[i]), &(this->st_mapIDs), sizeof(int32_t));
      }
      uint8_t maps_lengthT = *(inbuffer + offset++);
      if(maps_lengthT > maps_length)
        this->maps = (int32_t*)realloc(this->maps, maps_lengthT * sizeof(int32_t));
      offset += 3;
      maps_length = maps_lengthT;
      for( uint8_t i = 0; i < maps_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_maps;
      u_st_maps.base = 0;
      u_st_maps.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_maps.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_maps.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_maps.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_maps = u_st_maps.real;
      offset += sizeof(this->st_maps);
        memcpy( &(this->maps[i]), &(this->st_maps), sizeof(int32_t));
      }
      uint8_t imageIDs_lengthT = *(inbuffer + offset++);
      if(imageIDs_lengthT > imageIDs_length)
        this->imageIDs = (int32_t*)realloc(this->imageIDs, imageIDs_lengthT * sizeof(int32_t));
      offset += 3;
      imageIDs_length = imageIDs_lengthT;
      for( uint8_t i = 0; i < imageIDs_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_imageIDs;
      u_st_imageIDs.base = 0;
      u_st_imageIDs.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_imageIDs.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_imageIDs.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_imageIDs.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_imageIDs = u_st_imageIDs.real;
      offset += sizeof(this->st_imageIDs);
        memcpy( &(this->imageIDs[i]), &(this->st_imageIDs), sizeof(int32_t));
      }
      uint8_t images_lengthT = *(inbuffer + offset++);
      if(images_lengthT > images_length)
        this->images = (rtabmap::Bytes*)realloc(this->images, images_lengthT * sizeof(rtabmap::Bytes));
      offset += 3;
      images_length = images_lengthT;
      for( uint8_t i = 0; i < images_length; i++){
      offset += this->st_images.deserialize(inbuffer + offset);
        memcpy( &(this->images[i]), &(this->st_images), sizeof(rtabmap::Bytes));
      }
      uint8_t depthIDs_lengthT = *(inbuffer + offset++);
      if(depthIDs_lengthT > depthIDs_length)
        this->depthIDs = (int32_t*)realloc(this->depthIDs, depthIDs_lengthT * sizeof(int32_t));
      offset += 3;
      depthIDs_length = depthIDs_lengthT;
      for( uint8_t i = 0; i < depthIDs_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_depthIDs;
      u_st_depthIDs.base = 0;
      u_st_depthIDs.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_depthIDs.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_depthIDs.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_depthIDs.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_depthIDs = u_st_depthIDs.real;
      offset += sizeof(this->st_depthIDs);
        memcpy( &(this->depthIDs[i]), &(this->st_depthIDs), sizeof(int32_t));
      }
      uint8_t depths_lengthT = *(inbuffer + offset++);
      if(depths_lengthT > depths_length)
        this->depths = (rtabmap::Bytes*)realloc(this->depths, depths_lengthT * sizeof(rtabmap::Bytes));
      offset += 3;
      depths_length = depths_lengthT;
      for( uint8_t i = 0; i < depths_length; i++){
      offset += this->st_depths.deserialize(inbuffer + offset);
        memcpy( &(this->depths[i]), &(this->st_depths), sizeof(rtabmap::Bytes));
      }
      uint8_t depth2DIDs_lengthT = *(inbuffer + offset++);
      if(depth2DIDs_lengthT > depth2DIDs_length)
        this->depth2DIDs = (int32_t*)realloc(this->depth2DIDs, depth2DIDs_lengthT * sizeof(int32_t));
      offset += 3;
      depth2DIDs_length = depth2DIDs_lengthT;
      for( uint8_t i = 0; i < depth2DIDs_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_depth2DIDs;
      u_st_depth2DIDs.base = 0;
      u_st_depth2DIDs.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_depth2DIDs.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_depth2DIDs.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_depth2DIDs.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_depth2DIDs = u_st_depth2DIDs.real;
      offset += sizeof(this->st_depth2DIDs);
        memcpy( &(this->depth2DIDs[i]), &(this->st_depth2DIDs), sizeof(int32_t));
      }
      uint8_t depth2Ds_lengthT = *(inbuffer + offset++);
      if(depth2Ds_lengthT > depth2Ds_length)
        this->depth2Ds = (rtabmap::Bytes*)realloc(this->depth2Ds, depth2Ds_lengthT * sizeof(rtabmap::Bytes));
      offset += 3;
      depth2Ds_length = depth2Ds_lengthT;
      for( uint8_t i = 0; i < depth2Ds_length; i++){
      offset += this->st_depth2Ds.deserialize(inbuffer + offset);
        memcpy( &(this->depth2Ds[i]), &(this->st_depth2Ds), sizeof(rtabmap::Bytes));
      }
      uint8_t depthConstantIDs_lengthT = *(inbuffer + offset++);
      if(depthConstantIDs_lengthT > depthConstantIDs_length)
        this->depthConstantIDs = (int32_t*)realloc(this->depthConstantIDs, depthConstantIDs_lengthT * sizeof(int32_t));
      offset += 3;
      depthConstantIDs_length = depthConstantIDs_lengthT;
      for( uint8_t i = 0; i < depthConstantIDs_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_depthConstantIDs;
      u_st_depthConstantIDs.base = 0;
      u_st_depthConstantIDs.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_depthConstantIDs.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_depthConstantIDs.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_depthConstantIDs.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_depthConstantIDs = u_st_depthConstantIDs.real;
      offset += sizeof(this->st_depthConstantIDs);
        memcpy( &(this->depthConstantIDs[i]), &(this->st_depthConstantIDs), sizeof(int32_t));
      }
      uint8_t depthConstants_lengthT = *(inbuffer + offset++);
      if(depthConstants_lengthT > depthConstants_length)
        this->depthConstants = (float*)realloc(this->depthConstants, depthConstants_lengthT * sizeof(float));
      offset += 3;
      depthConstants_length = depthConstants_lengthT;
      for( uint8_t i = 0; i < depthConstants_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_depthConstants;
      u_st_depthConstants.base = 0;
      u_st_depthConstants.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_depthConstants.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_depthConstants.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_depthConstants.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_depthConstants = u_st_depthConstants.real;
      offset += sizeof(this->st_depthConstants);
        memcpy( &(this->depthConstants[i]), &(this->st_depthConstants), sizeof(float));
      }
      uint8_t localTransformIDs_lengthT = *(inbuffer + offset++);
      if(localTransformIDs_lengthT > localTransformIDs_length)
        this->localTransformIDs = (int32_t*)realloc(this->localTransformIDs, localTransformIDs_lengthT * sizeof(int32_t));
      offset += 3;
      localTransformIDs_length = localTransformIDs_lengthT;
      for( uint8_t i = 0; i < localTransformIDs_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_localTransformIDs;
      u_st_localTransformIDs.base = 0;
      u_st_localTransformIDs.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_localTransformIDs.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_localTransformIDs.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_localTransformIDs.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_localTransformIDs = u_st_localTransformIDs.real;
      offset += sizeof(this->st_localTransformIDs);
        memcpy( &(this->localTransformIDs[i]), &(this->st_localTransformIDs), sizeof(int32_t));
      }
      uint8_t localTransforms_lengthT = *(inbuffer + offset++);
      if(localTransforms_lengthT > localTransforms_length)
        this->localTransforms = (geometry_msgs::Transform*)realloc(this->localTransforms, localTransforms_lengthT * sizeof(geometry_msgs::Transform));
      offset += 3;
      localTransforms_length = localTransforms_lengthT;
      for( uint8_t i = 0; i < localTransforms_length; i++){
      offset += this->st_localTransforms.deserialize(inbuffer + offset);
        memcpy( &(this->localTransforms[i]), &(this->st_localTransforms), sizeof(geometry_msgs::Transform));
      }
      uint8_t poseIDs_lengthT = *(inbuffer + offset++);
      if(poseIDs_lengthT > poseIDs_length)
        this->poseIDs = (int32_t*)realloc(this->poseIDs, poseIDs_lengthT * sizeof(int32_t));
      offset += 3;
      poseIDs_length = poseIDs_lengthT;
      for( uint8_t i = 0; i < poseIDs_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_poseIDs;
      u_st_poseIDs.base = 0;
      u_st_poseIDs.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_poseIDs.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_poseIDs.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_poseIDs.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_poseIDs = u_st_poseIDs.real;
      offset += sizeof(this->st_poseIDs);
        memcpy( &(this->poseIDs[i]), &(this->st_poseIDs), sizeof(int32_t));
      }
      uint8_t poses_lengthT = *(inbuffer + offset++);
      if(poses_lengthT > poses_length)
        this->poses = (geometry_msgs::Pose*)realloc(this->poses, poses_lengthT * sizeof(geometry_msgs::Pose));
      offset += 3;
      poses_length = poses_lengthT;
      for( uint8_t i = 0; i < poses_length; i++){
      offset += this->st_poses.deserialize(inbuffer + offset);
        memcpy( &(this->poses[i]), &(this->st_poses), sizeof(geometry_msgs::Pose));
      }
      uint8_t constraintFromIDs_lengthT = *(inbuffer + offset++);
      if(constraintFromIDs_lengthT > constraintFromIDs_length)
        this->constraintFromIDs = (int32_t*)realloc(this->constraintFromIDs, constraintFromIDs_lengthT * sizeof(int32_t));
      offset += 3;
      constraintFromIDs_length = constraintFromIDs_lengthT;
      for( uint8_t i = 0; i < constraintFromIDs_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_constraintFromIDs;
      u_st_constraintFromIDs.base = 0;
      u_st_constraintFromIDs.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_constraintFromIDs.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_constraintFromIDs.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_constraintFromIDs.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_constraintFromIDs = u_st_constraintFromIDs.real;
      offset += sizeof(this->st_constraintFromIDs);
        memcpy( &(this->constraintFromIDs[i]), &(this->st_constraintFromIDs), sizeof(int32_t));
      }
      uint8_t constraintToIDs_lengthT = *(inbuffer + offset++);
      if(constraintToIDs_lengthT > constraintToIDs_length)
        this->constraintToIDs = (int32_t*)realloc(this->constraintToIDs, constraintToIDs_lengthT * sizeof(int32_t));
      offset += 3;
      constraintToIDs_length = constraintToIDs_lengthT;
      for( uint8_t i = 0; i < constraintToIDs_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_constraintToIDs;
      u_st_constraintToIDs.base = 0;
      u_st_constraintToIDs.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_constraintToIDs.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_constraintToIDs.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_constraintToIDs.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_constraintToIDs = u_st_constraintToIDs.real;
      offset += sizeof(this->st_constraintToIDs);
        memcpy( &(this->constraintToIDs[i]), &(this->st_constraintToIDs), sizeof(int32_t));
      }
      uint8_t constraintTypes_lengthT = *(inbuffer + offset++);
      if(constraintTypes_lengthT > constraintTypes_length)
        this->constraintTypes = (int32_t*)realloc(this->constraintTypes, constraintTypes_lengthT * sizeof(int32_t));
      offset += 3;
      constraintTypes_length = constraintTypes_lengthT;
      for( uint8_t i = 0; i < constraintTypes_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_constraintTypes;
      u_st_constraintTypes.base = 0;
      u_st_constraintTypes.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_constraintTypes.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_constraintTypes.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_constraintTypes.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_constraintTypes = u_st_constraintTypes.real;
      offset += sizeof(this->st_constraintTypes);
        memcpy( &(this->constraintTypes[i]), &(this->st_constraintTypes), sizeof(int32_t));
      }
      uint8_t constraints_lengthT = *(inbuffer + offset++);
      if(constraints_lengthT > constraints_length)
        this->constraints = (geometry_msgs::Transform*)realloc(this->constraints, constraints_lengthT * sizeof(geometry_msgs::Transform));
      offset += 3;
      constraints_length = constraints_lengthT;
      for( uint8_t i = 0; i < constraints_length; i++){
      offset += this->st_constraints.deserialize(inbuffer + offset);
        memcpy( &(this->constraints[i]), &(this->st_constraints), sizeof(geometry_msgs::Transform));
      }
     return offset;
    }

    const char * getType(){ return "rtabmap/MapData"; };
    const char * getMD5(){ return "7056370ee480a69ddc48b1f29aa5efc4"; };

  };

}
#endif