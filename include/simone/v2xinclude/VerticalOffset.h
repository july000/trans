/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DefPositionOffset"
 * 	found in "DefPositionOffset.asn"
 */

#ifndef	_VerticalOffset_H_
#define	_VerticalOffset_H_


#include <asn_application.h>

/* Including external dependencies */
#include "VertOffset-B07.h"
#include "VertOffset-B08.h"
#include "VertOffset-B09.h"
#include "VertOffset-B10.h"
#include "VertOffset-B11.h"
#include "VertOffset-B12.h"
#include "Elevation.h"
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum VerticalOffset_PR {
	VerticalOffset_PR_NOTHING,	/* No components present */
	VerticalOffset_PR_offset1,
	VerticalOffset_PR_offset2,
	VerticalOffset_PR_offset3,
	VerticalOffset_PR_offset4,
	VerticalOffset_PR_offset5,
	VerticalOffset_PR_offset6,
	VerticalOffset_PR_elevation
} VerticalOffset_PR;

/* VerticalOffset */
typedef struct VerticalOffset {
	VerticalOffset_PR present;
	union VerticalOffset_u {
		VertOffset_B07_t	 offset1;  //INTEGER (-64..63)
		VertOffset_B08_t	 offset2;  //INTEGER (-128..127) 
		VertOffset_B09_t	 offset3;  //INTEGER (-256..255)  
		VertOffset_B10_t	 offset4;  //INTEGER (-512..511) 
		VertOffset_B11_t	 offset5;  //INTEGER (-1024..1023) 
		VertOffset_B12_t	 offset6;  //INTEGER (-2048..2047) 
		Elevation_t	 elevation;   //INTEGER (-4096..61439)
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} VerticalOffset_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_VerticalOffset;
extern asn_CHOICE_specifics_t asn_SPC_VerticalOffset_specs_1;
extern asn_TYPE_member_t asn_MBR_VerticalOffset_1[7];
extern asn_per_constraints_t asn_PER_type_VerticalOffset_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _VerticalOffset_H_ */
#include <asn_internal.h>
