//
//  MapImage.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-04-25.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import <Foundation/Foundation.h>
#include <vector>

@interface MapImage : NSObject

    @property(nonatomic) BOOL displayed;
    @property(nonatomic) NSUInteger width;
    @property(nonatomic) NSUInteger height;
    @property(nonatomic) std::vector<unsigned char> data;

@end
