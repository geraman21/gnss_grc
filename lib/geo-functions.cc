double check_t(double time)
{
    double corrTime = time;
    int half_weeK = 302400;
    if (time > half_weeK)
        corrTime = time - (2 * half_weeK);
    else if (time < -half_weeK)
        corrTime = time + (2 * half_weeK);

    return corrTime;
}