class Contact(object):
    def __init__(self, data):
        self.p = data[:3]
        self.f = data[3:6]
        self.i = data[6]  # ID
