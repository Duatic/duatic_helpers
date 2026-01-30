import time
from typing import Union, List


def wait_for_topics(node, topic_names: Union[str, List[str]], timeout: float = 10.0):
    """
    Waits until the specified topic(s) are advertised in the ROS graph.

    Args:
        node: rclpy Node object used to query topics.
        topic_names: A single topic name or a list of topic names to look for.
        timeout: How long to wait before failing.

    Raises:
        AssertionError: If not all topics are found within the timeout.
    """
    # Normalize input to a set for easier tracking
    if isinstance(topic_names, str):
        missing_topics = {topic_names}
    else:
        missing_topics = set(topic_names)

    start = time.time()
    current_topics = []

    while time.time() - start < timeout:
        # Get list of currently advertised topics
        current_topics_info = node.get_topic_names_and_types()
        # Extract just the names
        current_topics = {name for name, _ in current_topics_info}

        # Remove found topics from our missing set
        missing_topics = missing_topics - current_topics

        # If no topics are missing, we are done
        if not missing_topics:
            return

        time.sleep(0.1)

    assert not missing_topics, (
        f"Topics {missing_topics} not advertised within {timeout} seconds. "
        f"Available topics: {current_topics}"
    )
