//
//  LoggerLevelTableViewCell.m
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-03-20.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import "LoggerLevelTableViewCell.h"

@implementation LoggerLevelTableViewCell

@synthesize node = _node;
@synthesize message = _message;

- (id)initWithStyle:(UITableViewCellStyle)style reuseIdentifier:(NSString *)reuseIdentifier
{
    self = [super initWithStyle:style reuseIdentifier:reuseIdentifier];
    if (self) {
        // Initialization code
    }
    return self;
}

- (void)setSelected:(BOOL)selected animated:(BOOL)animated
{
    [super setSelected:selected animated:animated];

    // Configure the view for the selected state
}

@end
