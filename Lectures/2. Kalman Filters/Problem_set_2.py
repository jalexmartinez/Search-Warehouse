import matrix

def kalman_filter(x, P, measurements):
    u = matrix.matrix([[0], [0]])  # external motion
    F = matrix.matrix([[1, 1], [0, 1]])  # next state function
    H = matrix.matrix([[1, 0]])  # measurement function
    R = matrix.matrix([[1]])  # measurement uncertainty
    I = matrix.matrix([[1, 0], [0, 1]])  # identity matrix

    for measure in measurements:

        #measurement step
        Z = matrix.matrix([[measure]])
        y = Z.transpose() - H * x
        S = H * P * H.transpose() + R
        K = P * H.transpose() * S.inverse()

        x_p = x + (K * y)
        P_p = (I - K * H) * P

        # Prediction step

        x = F * x_p + u
        P = F * P_p * F.transpose()

    return x, P