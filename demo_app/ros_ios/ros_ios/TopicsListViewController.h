//
//  TopicsListViewController.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-04-08.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>

@interface TopicsListViewController : UITableViewController

@property (nonatomic, strong) NSMutableArray * topics_name;
@property (nonatomic, strong) NSMutableArray * topics_type;

@end
