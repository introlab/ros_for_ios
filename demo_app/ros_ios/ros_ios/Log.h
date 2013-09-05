//
//  Log.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-09-04.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#ifndef ros_ios_log_h
#define ros_ios_log_h

#undef DEBUG
typedef NS_ENUM(NSInteger, LogLevel)
{
    DEBUG = 0,
    INFO,
    WARN,
    ERROR,
    FATAL,
    NB_LEVELS
};

#endif
