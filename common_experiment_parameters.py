# File with common experiment parameters.

def get_experiment_setup():
    '''
    Function which returns the standard parameters which need to be calibrated
    before an experiment.
    '''
    params = {
        'phasemask_width' : 1080,
        'phasemask_height' : 1080,
        'phasemask_position' : 2340,
        'slm_x_center': 558,# needs to be recalibrated if camera is moved.
        # This is the position of the 0th order of the SLM (ie where the trap)
        # with xm=ym=0 is located in camera coordinates
        'slm_y_center': 576,#605-29,
        'slm_to_pixel':5000000, # Basler
        #4550000.0, #thorlabs
        'x_comp':3.2e4, # compensates for shift in x,y coordninates when
        # changing z of trap with slm. Needs to be calibrated.
        'y_comp':9e4,
    }
    return params
