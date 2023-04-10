import numpy as np

class VBLR:
    def __init__(self, S_0=0, beta=1):
        self.beta = beta
        self.S_0_inv = 1/S_0
        self.m_n = None
        self.S_n_inv = None
        self.X = np.array([])
        self.y = np.array([])
    
    def fit(self, X, y):
        X = np.array(X).reshape(-1, 1)
        y = np.array(y).reshape(-1, 1)
        self.X = np.vstack([self.X, X])
        self.y = np.vstack([self.y, y])
        self.update_model()
    
    def update_model(self):
        n = self.X.shape[0]
        phi = np.hstack([np.ones((n, 1)), self.X])
        self.S_n_inv = self.S_0_inv + self.beta * phi.T @ phi
        self.m_n = self.beta * np.linalg.inv(self.S_n_inv) @ phi.T @ self.y
    
    def next_point(self, x=None):
        if x is None:
            x = int(self.X[-1]) + 1
        phi_x = np.array([1, x]).reshape(1, -1)
        sigma_n_squared = 1/self.beta + phi_x @ np.linalg.inv(self.S_n_inv) @ phi_x.T
        mu_n = phi_x @ self.m_n
        y = np.random.normal(mu_n, np.sqrt(sigma_n_squared))
        return x, y