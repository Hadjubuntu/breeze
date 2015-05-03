/*
 * LowPassFilter.h
 *
 *  Created on: May 3, 2015
 *      Author: adrien
 */

#ifndef MATH_LOWPASSFILTER_H_
#define MATH_LOWPASSFILTER_H_


#include <inttypes.h>
#include "Math.h"

class DigitalBiquadFilter
{
public:
    DigitalBiquadFilter() :
    _delay_element_1(0.0f),
    _delay_element_2(0.0f){}

    struct biquad_params {
        float cutoff_freq;
        float sample_freq;
        float a1;
        float a2;
        float b0;
        float b1;
        float b2;
    };

    float apply(float sample, const struct biquad_params &params);

    void reset() { _delay_element_1 = _delay_element_2 = 0.0f; }

    static void compute_params(float sample_freq, float cutoff_freq, biquad_params &ret);

private:
    float _delay_element_1;
    float _delay_element_2;
};



float DigitalBiquadFilter::apply(float sample, const struct biquad_params &params)
{
    if(params.cutoff_freq == 0 || params.sample_freq == 0) {
        return sample;
    }

    float delay_element_0 = sample - _delay_element_1 * params.a1 - _delay_element_2 * params.a2;
    if (isnan(delay_element_0) || isinf(delay_element_0)) {
        delay_element_0 = sample;
    }
    float output = delay_element_0 * params.b0 + _delay_element_1 * params.b1 + _delay_element_2 * params.b2;

    _delay_element_2 = _delay_element_1;
    _delay_element_1 = delay_element_0;

    return output;
}

void DigitalBiquadFilter::compute_params(float sample_freq, float cutoff_freq, biquad_params &ret)
{
    ret.cutoff_freq = cutoff_freq;
    ret.sample_freq = sample_freq;

    float fr = sample_freq/cutoff_freq;
    float ohm = tanf(M_PI/fr);
    float c = 1.0f+2.0f*cos(M_PI/4.0f)*ohm + ohm*ohm;

    ret.b0 = ohm*ohm/c;
    ret.b1 = 2.0f*ret.b0;
    ret.b2 = ret.b0;
    ret.a1 = 2.0f*(ohm*ohm-1.0f)/c;
    ret.a2 = (1.0f-2.0f*cos(M_PI/4.0f)*ohm+ohm*ohm)/c;
}


class LowPassFilter2p
{
public:
    LowPassFilter2p() { memset(&_params, 0, sizeof(_params)); }
    // constructor
    LowPassFilter2p(float sample_freq, float cutoff_freq) {
        // set initial parameters
        set_cutoff_frequency(sample_freq, cutoff_freq);
    }

    // change parameters
    void set_cutoff_frequency(float sample_freq, float cutoff_freq) {
        DigitalBiquadFilter::compute_params(sample_freq, cutoff_freq, _params);
    }

    // return the cutoff frequency
    float get_cutoff_freq(void) const {
        return _params.cutoff_freq;
    }

    float get_sample_freq(void) const {
        return _params.sample_freq;
    }

protected:
    struct DigitalBiquadFilter::biquad_params _params;
};

class LowPassFilter2pfloat : public LowPassFilter2p
{
public:
    LowPassFilter2pfloat() :
    LowPassFilter2p() {}

    LowPassFilter2pfloat(float sample_freq, float cutoff_freq):
    LowPassFilter2p(sample_freq,cutoff_freq) {}

    float apply(float sample) {
        return _filter.apply(sample, _params);
    }
private:
    DigitalBiquadFilter _filter;
};

class LowPassFilter2pVector3f : public LowPassFilter2p
{
public:
    LowPassFilter2pVector3f() :
    LowPassFilter2p() {}

    LowPassFilter2pVector3f(float sample_freq, float cutoff_freq) :
    LowPassFilter2p(sample_freq,cutoff_freq) {}

    Vector3f apply(const Vector3f &sample) {
        Vector3f ret;
        ret.x = _filter_x.apply(sample.x, _params);
        ret.y = _filter_y.apply(sample.y, _params);
        ret.z = _filter_z.apply(sample.z, _params);
        return ret;
    }

private:
    DigitalBiquadFilter _filter_x;
    DigitalBiquadFilter _filter_y;
    DigitalBiquadFilter _filter_z;
};



#endif /* MATH_LOWPASSFILTER_H_ */
