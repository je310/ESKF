import sympy as sp
from sympy.utilities.codegen import codegen

dPOS_IDX = (0)
dVEL_IDX = (dPOS_IDX + 3)
dTHETA_IDX = (dVEL_IDX + 3)
dAB_IDX = (dTHETA_IDX + 3)
dGB_IDX = (dAB_IDX + 3)
dSTATE_SIZE = (dGB_IDX + 3)

outfile_name = './unrolledJoseph.h'


def make_matrix(name, rows, cols):
    M = sp.zeros(rows, cols)
    for i in range(rows):
        for j in range(cols):
            M[i,j] = sp.Symbol('{}({},{})'.format(name, i, j))
    return M

def copy_lower_to_upper(M):
    for i in range(M.rows):
        for j in range(i, M.cols):
            M[i,j] = M[j,i]


dt = sp.symbols('dt')
F_x = sp.zeros(dSTATE_SIZE, dSTATE_SIZE)

# dPos row
F_x[dPOS_IDX:dPOS_IDX+3, dPOS_IDX:dPOS_IDX+3] = sp.eye(3)
F_x[dPOS_IDX:dPOS_IDX+3, dVEL_IDX:dVEL_IDX+3] = sp.eye(3) * dt
# dVel row
F_x[dVEL_IDX:dVEL_IDX+3, dVEL_IDX:dVEL_IDX+3] = sp.eye(3)
F_x[dVEL_IDX:dVEL_IDX+3, dTHETA_IDX:dTHETA_IDX+3] = make_matrix('dVel_dTheta', 3, 3)
F_x[dVEL_IDX:dVEL_IDX+3, dAB_IDX:dAB_IDX+3] = make_matrix('dVel_dAccelBias', 3, 3)
# dTheta row
F_x[dTHETA_IDX:dTHETA_IDX+3, dTHETA_IDX:dTHETA_IDX+3] = make_matrix('dTheta_dTheta', 3, 3)
F_x[dTHETA_IDX:dTHETA_IDX+3, dGB_IDX:dGB_IDX+3] = -sp.eye(3) * dt
# dAccelBias row
F_x[dAB_IDX:dAB_IDX+3, dAB_IDX:dAB_IDX+3] = sp.eye(3)
# dGyroBias row
F_x[dGB_IDX:dGB_IDX+3, dGB_IDX:dGB_IDX+3] = sp.eye(3)


P = make_matrix('Pin', dSTATE_SIZE, dSTATE_SIZE)
copy_lower_to_upper(P) # Exploit symmetric P for CSE

Pnew = F_x * P * F_x.T
copy_lower_to_upper(Pnew) # Exploit symmetric Pnew for CSE

# Generate common subexpressions
sub_expr, Pnew_cse = sp.cse(Pnew)
sub_expr_strs = ['const float {} = {};'.format(name, code) for name, code in sub_expr]

# Generate matrix expression
mat_strs = []
for res, expr in zip(
        sp.flatten(make_matrix('Pnew', Pnew.rows, Pnew.cols)),
        sp.flatten(Pnew_cse)):
    mat_strs.append('{} = {};'.format(res.name, sp.ccode(expr)))

# Write output
# Hint: Copy out to a text editor to edit with syntax, then copy back
header = """
#ifndef UNROLLEDFPFT_H
#define UNROLLEDFPFT_H
static inline void unrolledFPFt(
        const Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE>& Pin,
        Eigen::Matrix<float, dSTATE_SIZE, dSTATE_SIZE>& Pnew,
        const float dt,
        const Eigen::Matrix3f& dVel_dTheta,
        const Eigen::Matrix3f& dVel_dAccelBias,
        const Eigen::Matrix3f& dTheta_dTheta
        ) {
"""

body = '\n'.join(sub_expr_strs + mat_strs)

footer = """
}
#endif /* UNROLLEDFPFT_H */
"""

with open(outfile_name, 'w+') as outfile:
    outfile.write(header)
    outfile.write(body)
    outfile.write(footer)