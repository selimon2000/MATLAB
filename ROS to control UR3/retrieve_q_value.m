function q = retrieve_q_value(jointStateSubscriber)
    currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
    q = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
end