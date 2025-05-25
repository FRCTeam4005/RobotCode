@@ -0,0 +1,39 @@
#ifndef FIELD_CONSTANTS
#define FIELD_CONSTANTS

#include <units/length.h>


//Grid is what the cube shelf and cone ramp are called together
//Charge station is the balancing platform

//Outer is the furthest from the center of Field length wise
//Innter is the closest from the cneter of Field length wise

//the length of a piece or Field measurement will be along the longest measure ment of field dimension
//the width of a piece or Field measurement will be along the the shortest field dimension

// variable names will go  (Piece or Field)_(Piece Part or Fieldpoint_2_FieldPoint)_(l or w) can be left blank if it can only be measrued in one dimension

//OuterComm -> Outer Communication Boundry
//InnerComm -> Inter Communication Boundry


const auto Piece_ChargeStation_TopPanel_l = 48_in;
const auto Piece_ChargeStation_l = (96.75_in - 60.69_in) * 2;
const auto Piece_ChargeStation_RampPanel_l = 14.25_in;

const auto Field_InnerComm_2_GamePiece_TableSide = 85.13_in;
const auto Field_OuterComm_2_ChargingstationCenter = 96.75_in;
const auto Field_OuterComm_2_GamePiece = 244_in;
const auto Field_OuterComm_2_OuterChargeStation = 60.69_in;
const auto Field_OuterComm_2_InnerComm_Substation = Field_OuterComm_2_OuterChargeStation + Piece_ChargeStation_l;
const auto Field_OuterComm_2_InnerComm_TableSide = 96.75_in - Piece_ChargeStation_TopPanel_l;

// this is an estimation and cannot account for wheel slip, other factors, and may not be accurate at all
const auto Piece_ChargeStation_OffSet = Piece_chargeStation_RampPanel_l * 2  + Piece_ChargeStation_TopPanel_l;




#endif
No newline at end of file
