function dh = second_order_ODE(~, h)
    % Nonlinear coupled‑tank mass balances
    global u_in A1 A2 a12 a2 beta12 beta2 g

    % States (force ≥0)
    h1 = max(h(1),0);
    h2 = max(h(2),0);

    % Flows
    q12 = beta12 * a12 * sqrt(2*g * max(h1-h2,0));
    q2  = beta2   * a2  * sqrt(2*g * h2);

    % Level derivatives
    dh1 = (u_in - q12) / A1;
    dh2 = (q12   - q2 ) / A2;

    dh = [dh1; dh2];
end
