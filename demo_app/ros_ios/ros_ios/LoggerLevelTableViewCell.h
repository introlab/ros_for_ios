//
//  LoggerLevelTableViewCell.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-03-20.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>

@interface LoggerLevelTableViewCell : UITableViewCell
{
    UILabel * node;
    UILabel * message;
}

@property (nonatomic, strong) IBOutlet UILabel * node;
@property (nonatomic, strong) IBOutlet UILabel * message;

@end
