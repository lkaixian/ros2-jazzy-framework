from bt_orchestrator.policy import needs_arm


def test_help_me_carry_and_taking_orders_are_not_arm_requests():
    for goal in ('help me carry my bags', 'carry my bags and follow me until I say stop', 'take the orders of the customers',
                 'take an order from the customer'):
        assert not needs_arm(goal), goal


def test_real_arm_requests_still_need_the_arm():
    for goal in ('carry the cup to the table', 'take the cup from the door', 'bring me a glass', 'pick up the remote', 'open the door'):
        assert needs_arm(goal), goal
