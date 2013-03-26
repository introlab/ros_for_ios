//
//  LoggerTableViewCell.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-03-18.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>

@interface LoggerTableViewCell : UITableViewCell
{
    UILabel * type;
    UILabel * message;
}

@property (nonatomic, strong) IBOutlet UILabel * level;
@property (nonatomic, strong) IBOutlet UILabel * number;

@end
