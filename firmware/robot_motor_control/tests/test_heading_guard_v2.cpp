#include "../main/heading_guard.h"
#include <cassert>
#include <iostream>
int main(){
  using namespace HeadingGuard;
  Monitor fine;
  for(int i=0;i<60;++i)fine.update(0.12f,0.0f,-0.11f,-0.13f,0.1f);
  assert(!fine.blocked());
  Monitor reversed;
  for(int i=0;i<60;++i)reversed.update(0.12f,0.0f,0.12f,-0.13f,0.05f);
  assert(reversed.blocked() && reversed.reason()==OPPOSITE_RATE);
  // Latched even if sensor subsequently becomes correct.
  for(int i=0;i<100;++i)reversed.update(0.12f,0.0f,-0.1f,-0.1f,0.01f);
  assert(reversed.blocked());
  reversed.reset();assert(!reversed.blocked());
  Monitor yaw;
  yaw.update(.12f,0.0f,0.05f,0.05f,0.4f);
  assert(yaw.blocked() && yaw.reason()==EXCESSIVE_YAW_ERROR);
  Monitor curved;
  for(int i=0;i<100;++i)curved.update(0.10f,0.25f,0.30f,-0.15f,0.1f);
  assert(!curved.blocked()); // no sign checks on commanded arcs
  Monitor implausible;
  for(int i=0;i<30;++i)implausible.update(0.1f,0.0f,1.25f,0.01f,0.1f);
  assert(implausible.blocked() && implausible.reason()==IMPLAUSIBLE_RATE);
  std::cout<<"Heading V2 guard scenarios PASS\n";
}
