class Contact(object):
    def __init__(self, data):
        self.p = data[:3]
        self.f = data[3:6]
        self.i = data[6]  # ID

    def __repr__(self):
        # This is a quick and dirty method to check the contact list
        return "(C: %d)" % self.i
