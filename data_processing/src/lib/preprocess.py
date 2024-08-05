import geopandas as gpd
import numpy as np
import pandas as pd


def convert_to_dataframe(data):
    data = pd.json_normalize(data)
    if data.empty:
        raise ValueError('No rows to convert')

    return (
        data
        .pipe(_expand_FFT_column)
        .pipe(_convert_dtypes)
        .pipe(_drop_without_GPS_fix)
        .pipe(_convert_to_geodataframe)
    )


def _expand_FFT_column(df):
    new_columns = df['FFT'].apply(pd.Series).rename(columns=lambda i: f'FFT.{i*43}Hz')
    return df.drop(columns='FFT').join(new_columns)


def _convert_ddmm_to_degrees(ddmm):
    """
    Convert GPS lat and long format into decimalised degrees.

    ddmm format is an int with the first 2 or 3 digits representing the
    number of degrees, then the last 2 digits representing the minutes,
    which are at most 59.

    Examples:

    >>> convert_ddmm_to_degrees(3430)
    34.5

    >>> convert_ddmm_to_degrees(100.5)
    1.0083333333333333
    """
    degrees, minutes = np.divmod(ddmm, 100)
    return (degrees + minutes/60)


def _convert_dtypes(df):
    df = df.mask(df == '-')
    # This conversion is no longer required
    # df.loc[df['GPS.fix'] == 'False', 'GPS.fix'] = False  # legacy support (now a bool)
    df = df.astype({
        'datetime': 'datetime64[ns, UTC]',  # assume negligible difference between GPS time and UTC
        'GPS.fix': 'boolean',
        'GPS.location.lat': 'Float64',
        'GPS.location.long': 'Float64',
        'GPS.location.hdop': 'Float64',
        'GPS.location.alt': 'Float64',
        'GPS.speed': 'Float64',
        'GPS.angle': 'Float64',
        'GPS.satellites': 'Int64',
        'DHT.humidity': 'Float64',
        'DHT.temp': 'Float64',
        'DHT.heat_index': 'Float64',
        'MultiGas.no2': 'Int64',
        'MultiGas.c2h5ch': 'Int64',
        'MultiGas.voc': 'Int64',
        'MultiGas.co': 'Int64',
        'PM_Sensor.atmos_enviro.AE_1.0': 'Int64',
        'PM_Sensor.atmos_enviro.AE_2.5': 'Int64',
        'PM_Sensor.atmos_enviro.AE_10.0': 'Int64',
        'FFT.0Hz': 'Float64',
        'FFT.43Hz': 'Float64',
        'FFT.86Hz': 'Float64',
        'FFT.129Hz': 'Float64',
        'FFT.172Hz': 'Float64',
        'FFT.215Hz': 'Float64',
        'FFT.258Hz': 'Float64',
        'FFT.301Hz': 'Float64',
        'FFT.344Hz': 'Float64',
        'FFT.387Hz': 'Float64',
        'FFT.430Hz': 'Float64',
        'FFT.473Hz': 'Float64',
        'FFT.516Hz': 'Float64',
        'FFT.559Hz': 'Float64',
        'FFT.602Hz': 'Float64',
        'FFT.645Hz': 'Float64',
        'FFT.688Hz': 'Float64',
        'FFT.731Hz': 'Float64',
        'FFT.774Hz': 'Float64',
        'FFT.817Hz': 'Float64',
        'FFT.860Hz': 'Float64',
        'FFT.903Hz': 'Float64',
        'FFT.946Hz': 'Float64',
        'FFT.989Hz': 'Float64',
        'FFT.1032Hz': 'Float64',
        'FFT.1075Hz': 'Float64',
        'FFT.1118Hz': 'Float64',
        'FFT.1161Hz': 'Float64',
        'FFT.1204Hz': 'Float64',
        'FFT.1247Hz': 'Float64',
        'FFT.1290Hz': 'Float64',
        'FFT.1333Hz': 'Float64',
        'FFT.1376Hz': 'Float64',
        'FFT.1419Hz': 'Float64',
        'FFT.1462Hz': 'Float64',
        'FFT.1505Hz': 'Float64',
        'FFT.1548Hz': 'Float64',
    })
    # These conversions are no longer required (lat/lon are correctly-signed floats in degrees)
    # df['GPS.location.lat'] = df['GPS.location.lat'].pipe(_convert_ddmm_to_degrees)
    # df['GPS.location.long'] = -df['GPS.location.long'].pipe(_convert_ddmm_to_degrees)  # assume degrees WEST
    return df


def _drop_without_GPS_fix(df):
    return df.loc[df['GPS.fix']].dropna(subset=['GPS.location.lat', 'GPS.location.long'])


def _convert_to_geodataframe(df):
    return gpd.GeoDataFrame(
        df,
        geometry=gpd.points_from_xy(
            df['GPS.location.long'],
            df['GPS.location.lat'],
            crs='epsg:4326',  # assumed from GPS sensor
        ),
    )
