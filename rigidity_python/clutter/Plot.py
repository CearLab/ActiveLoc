import numpy as np
import matplotlib.pyplot as plt
import plotly.graph_objs as go

def plot_3D_vis(X, Y, Z, Xtitle='X', Ytitle='Y', Ztitle='Z'):

    sc = go.Scatter3d(
        x=X,
        y=Y,
        z=Z,
        mode='markers',
        marker=dict(
            size=5,
            color=Z,
            colorscale='plasma',
            opacity=0.7,
            line=dict(
                color=Z,
                width=0.5
            ),
            colorbar=dict(title='Z Value')
        )
    )

    layout = go.Layout(
        title='3D Scatter plot',
        scene=dict(
            xaxis_title=Xtitle,
            yaxis_title=Ytitle,
            zaxis=dict(title=Ztitle, range=[min(Z), max(Z)])
        ),
        width=1000,
        height=800
    )

    fig = go.Figure(data=[sc], layout=layout)
    fig.show()
    return 0