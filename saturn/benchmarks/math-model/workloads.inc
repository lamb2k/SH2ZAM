/* Included twice so both versions receive the same batch/output work. MIT. */
uint32_t RANGE(uint32_t first,uint32_t count) {
 if(first>MATH_BATCH || count>MATH_BATCH-first) return 0;
 uint32_t (*out)[MATH_WORDS]=OUTPUT;
 switch(math_operation) {
 case M_MUL:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; out[i][0]=(uint32_t)CALL(mul)(p->a,p->b); }
  break;
 case M_DIV:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; out[i][0]=(uint32_t)CALL(div)(p->a,p->b); }
  break;
 case M_MULDIV:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; out[i][0]=(uint32_t)CALL(muldiv)(p->a,p->b,p->c); }
  break;
 case M_RECIP:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; out[i][0]=(uint32_t)CALL(recip)(p->b); }
  break;
 case M_SQRT:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; out[i][0]=(uint32_t)CALL(sqrt)(p->root); }
  break;
 case M_RSQRT:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; out[i][0]=(uint32_t)CALL(rsqrt)(p->root); }
  break;
 case M_LERP:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; out[i][0]=(uint32_t)CALL(lerp)(p->a,p->b,p->t); }
  break;
 case M_SMOOTHSTEP:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; out[i][0]=(uint32_t)CALL(smoothstep)(p->t); }
  break;
 case M_VEC2_DOT:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; out[i][0]=(uint32_t)CALL(vec2_dot)(p->v2,p->w2); }
  break;
 case M_VEC3_DOT:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; out[i][0]=(uint32_t)CALL(vec3_dot)(p->v3,p->w3); }
  break;
 case M_VEC4_DOT:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; out[i][0]=(uint32_t)CALL(vec4_dot)(p->v4,p->w4); }
  break;
 case M_VEC2_CROSS:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; out[i][0]=(uint32_t)CALL(vec2_cross)(p->v2,p->w2); }
  break;
 case M_VEC3_CROSS:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; sh2z_vec3_t result=CALL(vec3_cross)(p->v3,p->w3); memcpy(out[i],&result,sizeof result); }
  break;
 case M_VEC2_LENGTH:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; out[i][0]=(uint32_t)CALL(vec2_length)(p->v2); }
  break;
 case M_VEC2_NORMALIZE:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; sh2z_vec2_t result=CALL(vec2_normalize)(p->v2); memcpy(out[i],&result,sizeof result); }
  break;
 case M_VEC3_LENGTH:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; out[i][0]=(uint32_t)CALL(vec3_length)(p->v3); }
  break;
 case M_VEC3_NORMALIZE:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; sh2z_vec3_t result=CALL(vec3_normalize)(p->v3); memcpy(out[i],&result,sizeof result); }
  break;
 case M_VEC4_LENGTH:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; out[i][0]=(uint32_t)CALL(vec4_length)(p->v4); }
  break;
 case M_VEC4_NORMALIZE:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; sh2z_vec4_t result=CALL(vec4_normalize)(p->v4); memcpy(out[i],&result,sizeof result); }
  break;
 case M_MAT22_MULTIPLY:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; sh2z_mat22_t result; CALL(mat22_multiply)(&result,&p->a22,&p->b22); memcpy(out[i],&result,sizeof result); }
  break;
 case M_MAT33_MULTIPLY:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; sh2z_mat33_t result; CALL(mat33_multiply)(&result,&p->a33,&p->b33); memcpy(out[i],&result,sizeof result); }
  break;
 case M_MAT34_MULTIPLY:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; sh2z_mat34_t result; CALL(mat34_multiply)(&result,&p->a34,&p->b34); memcpy(out[i],&result,sizeof result); }
  break;
 case M_MAT44_MULTIPLY:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; sh2z_mat44_t result; CALL(mat44_multiply)(&result,&p->a44,&p->b44); memcpy(out[i],&result,sizeof result); }
  break;
 case M_MAT22_TRANSFORM:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; sh2z_vec2_t result=CALL(mat22_transform)(&p->a22,p->v2); memcpy(out[i],&result,sizeof result); }
  break;
 case M_MAT33_TRANSFORM:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; sh2z_vec3_t result=CALL(mat33_transform)(&p->a33,p->v3); memcpy(out[i],&result,sizeof result); }
  break;
 case M_MAT34_TRANSFORM_POINT:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; sh2z_vec3_t result=CALL(mat34_transform_point)(&p->a34,p->v3); memcpy(out[i],&result,sizeof result); }
  break;
 case M_MAT34_TRANSFORM_DIRECTION:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; sh2z_vec3_t result=CALL(mat34_transform_direction)(&p->a34,p->v3); memcpy(out[i],&result,sizeof result); }
  break;
 case M_MAT44_TRANSFORM:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; sh2z_vec4_t result=CALL(mat44_transform)(&p->a44,p->v4); memcpy(out[i],&result,sizeof result); }
  break;
 case M_QUAT_MULTIPLY:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; sh2z_quat_t result=CALL(quat_multiply)(p->q,p->r); memcpy(out[i],&result,sizeof result); }
  break;
 case M_QUAT_NORMALIZE:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; sh2z_quat_t result=CALL(quat_normalize)(p->q); memcpy(out[i],&result,sizeof result); }
  break;
 case M_QUAT_NLERP:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; sh2z_quat_t result=CALL(quat_nlerp)(p->q,p->r,p->t); memcpy(out[i],&result,sizeof result); }
  break;
 case M_QUAT_TO_MAT34:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; sh2z_mat34_t result=CALL(quat_to_mat34)(p->q); memcpy(out[i],&result,sizeof result); }
  break;
 case M_QUAT_ROTATE:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; sh2z_vec3_t result=CALL(quat_rotate)(p->q,p->v3); memcpy(out[i],&result,sizeof result); }
  break;
 case M_QUAT_AXIS_ANGLE:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; sh2z_quat_t result=CALL(quat_axis_angle)(p->v3,p->angle); memcpy(out[i],&result,sizeof result); }
  break;
 case M_SIN:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; out[i][0]=(uint32_t)CALL(sin)(p->angle); }
  break;
 case M_COS:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; out[i][0]=(uint32_t)CALL(cos)(p->angle); }
  break;
 case M_SINCOS:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; sh2z_fx_t s,c; CALL(sincos)(p->angle,&s,&c); out[i][0]=(uint32_t)s; out[i][1]=(uint32_t)c; }
  break;
 case M_TAN:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; out[i][0]=(uint32_t)CALL(tan)(p->angle); }
  break;
 case M_ATAN2:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; out[i][0]=(uint32_t)CALL(atan2)(p->a,p->b); }
  break;
 case M_ATAN:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; out[i][0]=(uint32_t)CALL(atan)(p->a); }
  break;
 case M_ASIN:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; out[i][0]=(uint32_t)CALL(asin)(p->unit); }
  break;
 case M_ACOS:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; out[i][0]=(uint32_t)CALL(acos)(p->unit); }
  break;
 case M_PROJECT_CLIP:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; sh2z_screen_t result={123,-456}; out[i][0]=(uint32_t)CALL(project_clip)(&result,p->clip,320,224); out[i][1]=(uint32_t)(int32_t)result.x; out[i][2]=(uint32_t)(int32_t)result.y; }
  break;
 case M_MAT34_INVERSE_RIGID:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; sh2z_mat34_t result; memset(&result,0x5a,sizeof result); out[i][0]=(uint32_t)CALL(mat34_inverse_rigid)(&result,&p->rigid); memcpy(out[i]+1,&result,sizeof result); }
  break;
 case M_MAT34_LOOK_AT:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; sh2z_mat34_t result; memset(&result,0x5a,sizeof result); out[i][0]=(uint32_t)CALL(mat34_look_at)(&result,p->v3,p->w3,((sh2z_vec3_t){0,SH2Z_ONE,0})); memcpy(out[i]+1,&result,sizeof result); }
  break;
 case M_MAT44_PERSPECTIVE:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; sh2z_mat44_t result; memset(&result,0x5a,sizeof result); out[i][0]=(uint32_t)CALL(mat44_perspective)(&result,p->fov,p->aspect,p->near_z,p->far_z); memcpy(out[i]+1,&result,sizeof result); }
  break;
 case M_SPHERE_INTERSECTS:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; out[i][0]=(uint32_t)CALL(sphere_intersects)(p->s1,p->s2); }
  break;
 case M_SPHERE_INTERSECTS_AABB:
  for(uint32_t i=first;i<first+count;i++) { const struct math_input *p=&math_inputs[i]; out[i][0]=(uint32_t)CALL(sphere_intersects_aabb)(p->s1,p->box); }
  break;
 default:return 0;
 }
 return 1;
}
