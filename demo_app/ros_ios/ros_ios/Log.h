//
//  Log.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-03-19.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//


#import <Foundation/Foundation.h>
#undef DEBUG

@interface Log : NSObject

    typedef NS_ENUM(NSInteger, LogLevel) {
        DEBUG = 0,
        INFO,
        WARN,
        ERROR,
        FATAL,
        NB_LEVEL
    };

    @property(nonatomic) LogLevel level;
    @property(nonatomic,copy) NSString * stamp;
    @property(nonatomic,copy) NSString * message;
    @property(nonatomic,copy) NSString * node;

@end
