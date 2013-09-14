//
//  TopicViewCell.m
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-04-08.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import "TopicTableViewCell.h"

@implementation TopicTableViewCell

@synthesize name = _name;
@synthesize data_type = _data_type;

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
