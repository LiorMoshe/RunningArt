
class Segment(object):

    def __init__(self, start, end):
        self.start = start
        self.end = end

    def is_reverse(self, other):
        """
        Check if the given segment is the reverse of the previous one.
        :param other:
        :return:
        """
        return self.start == other.end and self.end == other.start

    def __hash__(self):
        return hash((self.start, self.end))

    def __eq__(self, other):
        """
        Two segments are regarded to one another as equal if they have similar path
        or they are the reverse of one another.
        :param other:
        :return:
        """
        return self.start == other.start and self.end == other.end or self.is_reverse(other)