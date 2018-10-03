//
// author Kazys Stepanas
//
#ifndef DEBUG_IDS_H_
#define DEBUG_IDS_H_

namespace ohm
{
  enum Category
  {
    CAT_Default,
    CAT_Map,
    // Parent category for below.
    CAT_Populate,
    CAT_Rays,
    CAT_FreeCells,
    CAT_OccupiedCells,
    CAT_Info
  };

  enum Resource
  {
    RES_Map = 1,
    RES_MapMesh = 1,  // OK to be the same as RES_Map. One is object, the other is resource.
  };
}

#endif // DEBUG_IDS_H_