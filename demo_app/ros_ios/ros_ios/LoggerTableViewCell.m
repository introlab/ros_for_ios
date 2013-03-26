//
//  LoggerTableViewCell.m
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-03-18.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import "LoggerTableViewCell.h"

@implementation LoggerTableViewCell

@synthesize level = _level;
@synthesize number = _number;

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
