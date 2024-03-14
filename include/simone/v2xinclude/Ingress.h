/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "BusStation"
 * 	found in "BusStation.asn"
 */

#ifndef	_Ingress_H_
#define	_Ingress_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct PositionOffsetLLV;

/* Ingress */
typedef struct Ingress {
	long	 id;
	long	 radius;
	struct path {
		A_SEQUENCE_OF(struct PositionOffsetLLV) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} path;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} Ingress_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_Ingress;
extern asn_SEQUENCE_specifics_t asn_SPC_Ingress_specs_1;
extern asn_TYPE_member_t asn_MBR_Ingress_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "PositionOffsetLLV.h"

#endif	/* _Ingress_H_ */
#include <asn_internal.h>
