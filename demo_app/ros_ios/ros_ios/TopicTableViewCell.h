//
//  TopicTableViewCell.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-04-08.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>

@interface TopicTableViewCell : UITableViewCell
{
    UILabel * name;
    UILabel * data_type;
}

@property (nonatomic, strong) IBOutlet UILabel * name;
@property (nonatomic, strong) IBOutlet UILabel * data_type;

@end
