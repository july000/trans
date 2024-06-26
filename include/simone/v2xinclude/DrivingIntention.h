/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IPPR"
 * 	found in "IPPR.asn"
 */

#ifndef	_DrivingIntention_H_
#define	_DrivingIntention_H_


#include <asn_application.h>

/* Including external dependencies */
#include <OCTET_STRING.h>
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum DrivingIntention_PR {
	DrivingIntention_PR_NOTHING,	/* No components present */
	DrivingIntention_PR_goStraight,
	DrivingIntention_PR_turnLeft,
	DrivingIntention_PR_turnRight,
	DrivingIntention_PR_turnRound
	/* Extensions may appear below */
	
} DrivingIntention_PR;

/* DrivingIntention */
typedef struct DrivingIntention {
	DrivingIntention_PR present;
	union DrivingIntention_u {
		OCTET_STRING_t	 goStraight;
		OCTET_STRING_t	 turnLeft;
		OCTET_STRING_t	 turnRight;
		OCTET_STRING_t	 turnRound;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} DrivingIntention_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_DrivingIntention;
extern asn_CHOICE_specifics_t asn_SPC_DrivingIntention_specs_1;
extern asn_TYPE_member_t asn_MBR_DrivingIntention_1[4];
extern asn_per_constraints_t asn_PER_type_DrivingIntention_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _DrivingIntention_H_ */
#include <asn_internal.h>
