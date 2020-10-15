#pragma once


#ifndef _H_DEMO_SETUP_
#define _H_DEMO_SETUP_


#define DEGREE  (M_PI/180)

#define TELEOPERATION

#if defined(NULL_MOTION)
#define __IndyRP2__
#else
#define __Indy7__
#endif

#endif /*_H_DEMO_SETUP*/