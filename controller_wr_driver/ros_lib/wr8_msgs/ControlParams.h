#ifndef _ROS_SERVICE_ControlParams_h
#define _ROS_SERVICE_ControlParams_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace wr8_msgs
{

static const char CONTROLPARAMS[] = "wr8_msgs/ControlParams";

  class ControlParamsRequest : public ros::Msg
  {
    public:
      typedef bool _request_only_type;
      _request_only_type request_only;
      typedef int32_t _esc_min_dc_offset_type;
      _esc_min_dc_offset_type esc_min_dc_offset;
      typedef int32_t _esc_max_dc_offset_type;
      _esc_max_dc_offset_type esc_max_dc_offset;

    ControlParamsRequest():
      request_only(0),
      esc_min_dc_offset(0),
      esc_max_dc_offset(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_request_only;
      u_request_only.real = this->request_only;
      *(outbuffer + offset + 0) = (u_request_only.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->request_only);
      union {
        int32_t real;
        uint32_t base;
      } u_esc_min_dc_offset;
      u_esc_min_dc_offset.real = this->esc_min_dc_offset;
      *(outbuffer + offset + 0) = (u_esc_min_dc_offset.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_esc_min_dc_offset.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_esc_min_dc_offset.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_esc_min_dc_offset.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->esc_min_dc_offset);
      union {
        int32_t real;
        uint32_t base;
      } u_esc_max_dc_offset;
      u_esc_max_dc_offset.real = this->esc_max_dc_offset;
      *(outbuffer + offset + 0) = (u_esc_max_dc_offset.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_esc_max_dc_offset.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_esc_max_dc_offset.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_esc_max_dc_offset.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->esc_max_dc_offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_request_only;
      u_request_only.base = 0;
      u_request_only.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->request_only = u_request_only.real;
      offset += sizeof(this->request_only);
      union {
        int32_t real;
        uint32_t base;
      } u_esc_min_dc_offset;
      u_esc_min_dc_offset.base = 0;
      u_esc_min_dc_offset.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_esc_min_dc_offset.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_esc_min_dc_offset.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_esc_min_dc_offset.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->esc_min_dc_offset = u_esc_min_dc_offset.real;
      offset += sizeof(this->esc_min_dc_offset);
      union {
        int32_t real;
        uint32_t base;
      } u_esc_max_dc_offset;
      u_esc_max_dc_offset.base = 0;
      u_esc_max_dc_offset.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_esc_max_dc_offset.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_esc_max_dc_offset.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_esc_max_dc_offset.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->esc_max_dc_offset = u_esc_max_dc_offset.real;
      offset += sizeof(this->esc_max_dc_offset);
     return offset;
    }

    const char * getType(){ return CONTROLPARAMS; };
    const char * getMD5(){ return "788f1ec6d5a7f27f03a74b0c08391f35"; };

  };

  class ControlParamsResponse : public ros::Msg
  {
    public:
      uint32_t params_length;
      typedef float _params_type;
      _params_type st_params;
      _params_type * params;

    ControlParamsResponse():
      params_length(0), params(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->params_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->params_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->params_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->params_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->params_length);
      for( uint32_t i = 0; i < params_length; i++){
      union {
        float real;
        uint32_t base;
      } u_paramsi;
      u_paramsi.real = this->params[i];
      *(outbuffer + offset + 0) = (u_paramsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_paramsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_paramsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_paramsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->params[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t params_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      params_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      params_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      params_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->params_length);
      if(params_lengthT > params_length)
        this->params = (float*)realloc(this->params, params_lengthT * sizeof(float));
      params_length = params_lengthT;
      for( uint32_t i = 0; i < params_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_params;
      u_st_params.base = 0;
      u_st_params.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_params.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_params.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_params.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_params = u_st_params.real;
      offset += sizeof(this->st_params);
        memcpy( &(this->params[i]), &(this->st_params), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return CONTROLPARAMS; };
    const char * getMD5(){ return "8e22f8c3368a715022fd214d9775704d"; };

  };

  class ControlParams {
    public:
    typedef ControlParamsRequest Request;
    typedef ControlParamsResponse Response;
  };

}
#endif
