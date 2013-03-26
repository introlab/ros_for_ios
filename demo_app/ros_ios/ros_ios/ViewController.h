//
//  ViewController.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>

@interface ViewController : UIViewController

@property (weak, nonatomic) IBOutlet UITextField *ip_text_field;

- (IBAction)ip_edit_ended:(id)sender;

+ (NSString *)getIPAddress;

+ (BOOL)isValidIp:(NSString*)string;

@end
