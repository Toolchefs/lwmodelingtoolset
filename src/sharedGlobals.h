//
//  sharedGlobals.h
//  LatticeModeler
//
//  Created by Daniele Federico on 24/03/15.
//
//

#ifndef SharedGlobals_h
#define SharedGlobals_h

#include <lwhost.h>

#include "vector3F.h"

/* global variables typedef function */

extern LWMessageFuncs *msgf;
extern LWStateQueryFuncs *query;
extern LWXPanelFuncs *xpanf;

/* casting typedef function */
typedef void*(*get_function)(void*, unsigned int);
typedef LWXPRefreshCode(*set_function)(void*, unsigned int, void*);

typedef void(*done_function)(void*);
typedef int(*count_function)(void*, LWToolEvent*);
typedef void (*draw_function)(void*, LWWireDrawAccess*);
typedef int(*dirty_function)(void*);
typedef void(*up_function)(void*, LWToolEvent*);
typedef const char* (*help_function)(void*, LWToolEvent*);
typedef void (*move_function)(void*, LWToolEvent*);
typedef int (*down_function)(void*, LWToolEvent*);
typedef void* (*panel_function)(void*);
typedef void (*event_function)(void*, int);
typedef const char*(*build_function)(void*, MeshEditOp*);
typedef int(*test_function)(void*);
typedef void(*end_function)(void*, int);

typedef EDError (*fastPntScan_function)(void*, LWPntID);
typedef EDError (*fastPolyScan_function)(void*, LWPolID);
typedef EDError (*fastEdgeScan_function)(void*, LWEdgeID);
typedef EDError (*edgeScan_function)(void*, const EDEdgeInfo*);

typedef int (*activate_function)(int, void* (*)(const char*, int), void*, void*);

#endif
