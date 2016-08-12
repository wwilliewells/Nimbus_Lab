function [ output_args ] = test_run_breakdown( input_args )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% a task was added/missed
crash_no_qci = [509 506 505 447 439 399 366 343 296 264]
% unexplained jenny 107

% overshoots in translation or rotation and a task was added/missed
crash_both = [491 477 476 471 467 466 462 458 457 442 415 409 404 198 194 185 180 164 159 154 142 141 132 128 123 120 115 111 107]

% overshoots in translation or rotation
crash_no_task = [497 496 492 486 472 461 453 441 427 423 414  395 390 385 379 378 193 181 176 172 167 163 160 147 133]

crash_other_fault = [13 14 64 155 190]


not_qci = [losc,et,estate,state,task]
not_state = [losc,mt,mPID,qci,ss,sp,task,et,estate]
not_pose = [mt,mPID,qci,ss,sp]

% look at
crash_lu = [390 399 427];
herbie_lu = [20,76];
jenny_lu = [25,46];
unl_comp_sci_lu = [5];

end

