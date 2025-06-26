from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
import scipy.linalg
import numpy as np
from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, sqrt

def export_heron_model() -> AcadosModel:
    model_name = 'heron'
    # constants

    Xu = 0.24
    Xuu = 0.006
    b1 = 0.00078831
    Yv = 0.1284
    Yr = 0.56798
    Nr = 0.55294
    b2 = 0.0011599
    b3 = 0.73167

    # set up states & controls
    xn   = SX.sym('xn')
    yn   = SX.sym('yn')
    psi  = SX.sym('psi')
    u    = SX.sym('u')
    v    = SX.sym('v')
    r    = SX.sym('r')

    delta  = SX.sym('delta')
    F  = SX.sym('F')

    states = vertcat(xn, yn, psi, u, v, r, delta, F)

    delta_d  = SX.sym('delta_d')
    F_d  = SX.sym('F_d')
    inputs  = vertcat(delta_d, F_d)

    # xdot
    xn_dot  = SX.sym('xn_dot')
    yn_dot  = SX.sym('yn_dot')
    psi_dot = SX.sym('psi_dot')
    u_dot   = SX.sym('u_dot')
    v_dot   = SX.sym('v_dot')
    r_dot   = SX.sym('r_dot')
    delta_dot   = SX.sym('delta_dot')
    F_dot   = SX.sym('F_dot')

    states_dot = vertcat(xn_dot, yn_dot, psi_dot, u_dot, v_dot, r_dot, delta_dot, F_dot)

    eps = 0.00001
    # dynamics
    f_expl = vertcat(u*cos(psi) - v*sin(psi),
                     u*sin(psi) + v*cos(psi),
                     r,
                     ( - Xu*u + b1*F*F*cos(b2*delta)),
                     ( -Yv*v - Yr*r + b1*F*F*sin(b2*delta)),
                     ( - Nr*r - b3*b1*F*F*sin(b2*delta)),
                     delta_d,
                     F_d
                     )

    f_impl = states_dot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = states
    model.xdot = states_dot
    model.u = inputs
    model.name = model_name

    # store meta information
    model.x_labels = ['$x$ [m]', '$y$ [m]',  '$psi$ [rad]',  '$u$ [m/s]', '$v$ [m/s]', '$r$ [rad/s]', '$delta$ [N]', '$F$ [N]']
    model.u_labels = ['$n_1_d$ [N/s]', '$n_2_d$ [N/s]']
    model.t_label = '$t$ [s]'

    return model


def setup_trajectory_tracking(x0, N_horizon, Tf):
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    # set model
    model = export_heron_model()
    ocp.model = model

    nx = model.x.rows()
    nu = model.u.rows()
    ny = nx + nu
    ny_e = nx

    ocp.dims.N = N_horizon

    # set cost module
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'

    Q_mat = 2*np.diag([1, 1, 1e-2, 1e1, 1e1, 1-1, 1e-4, 1e-4])
    R_mat = 2*np.diag([1e-3, 1e-3])

    ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)
    ocp.cost.W_e = Q_mat

    ocp.model.cost_y_expr = vertcat(model.x, model.u)
    ocp.model.cost_y_expr_e = model.x
    ocp.cost.yref  = np.zeros((ny, ))
    ocp.cost.yref_e = np.zeros((ny_e, ))

    ocp.constraints.x0 = x0

    # set up parameters
    ox1 = SX.sym('ox1') 
    oy1 = SX.sym('oy1') 
    or1 = SX.sym('or1') 
    ox2 = SX.sym('ox2') 
    oy2 = SX.sym('oy2') 
    or2 = SX.sym('or2') 
    
    p = vertcat(ox1, oy1, or1, 
                ox2, oy2, or2)
    
    ocp.model.p = p 
    ocp.parameter_values = np.array([0.0, 0.0, 0.0, 
                                     0.0, 0.0, 0.0])

    num_obs = 2
    ocp.constraints.uh = 1e10 * np.ones(num_obs)
    ocp.constraints.lh = np.zeros(num_obs)
    h_expr = SX.zeros(num_obs,1)
    h_expr[0] = (model.x[0]-ox1) ** 2 + (model.x[1] - oy1) ** 2 - or1**2
    h_expr[1] = (model.x[0]-ox2) ** 2 + (model.x[1] - oy2) ** 2 - or2**2
    ocp.model.con_h_expr = h_expr

    ocp.constraints.idxsh = np.array([0,1])
    ocp.constraints.idxsh_e = np.array([0,1])
    Zh = 1e5 * np.ones(num_obs)
    zh = 1e5 * np.ones(num_obs)
    ocp.cost.zl = zh
    ocp.cost.zu = zh
    ocp.cost.Zl = Zh
    ocp.cost.Zu = Zh
    ocp.cost.zl_e = zh
    ocp.cost.zu_e = zh
    ocp.cost.Zl_e = Zh
    ocp.cost.Zu_e = Zh

    # copy for terminal
    ocp.constraints.uh_e = ocp.constraints.uh
    ocp.constraints.lh_e = ocp.constraints.lh
    ocp.model.con_h_expr_e = ocp.model.con_h_expr

    # set constraints
    ocp.constraints.lbu = np.array([-100.0,-10.0])
    ocp.constraints.ubu = np.array([+100.0,+10.0])
    ocp.constraints.idxbu = np.array([0, 1])

    ocp.constraints.lbx = np.array([-300.0, 0])
    ocp.constraints.ubx = np.array([300.0, 100.0])
    ocp.constraints.idxbx = np.array([6, 7])

    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'IRK'
    ocp.solver_options.sim_method_newton_iter = 50
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.qp_solver_cond_N = N_horizon

    # set prediction horizon
    ocp.solver_options.tf = Tf
    solver_json = 'acados_ocp_' + model.name + '.json'
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file = solver_json)
    # create an integrator with the same settings as used in the OCP solver.
    # acados_integrator = AcadosSimSolver(ocp, json_file = solver_json)

    return acados_ocp_solver


